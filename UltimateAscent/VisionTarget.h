#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="VisionTarget.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     VisionTarget class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_TARGET
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Target"

typedef struct _targetInfo
{
    float distance;
    float height;
    float hAngle;
    float vAngle;
} TARGETINFO, *PTARGETINFO;

/**
 * This module defines and implements the VisionTarget subsytem. The
 * VisionTarget subsystem consists of an IP camera. It takes a snapshot
 * from the camera and detect the target rectangles in the snapshot. It
 * returns a list of target objects.
 */
class VisionTarget: public CoopTask
{
private:
    DashboardDataFormat*m_dashboardDataFormat;
    AxisCamera&         m_camera;
    Threshold           m_colorThresholds;
    VisionTask          m_visionTask;
    ParticleFilterCriteria2 m_filterCriteria[2];
    
    KalmanFilter        m_frameRateFilter;
    
    int                 m_targetID;
    int                 m_lastX;
    int                 m_lastY;

public:
    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    VisionTarget(
        DashboardDataFormat *dashboardDataFormat
        ): m_dashboardDataFormat(dashboardDataFormat)
         , m_camera(AxisCamera::GetInstance(CAMERA_IP))
        //
        // Good green values: RGB(0/100, 80/140, 40/140)
        // Obtained from camera snapshot in mspaint.
        //
         , m_colorThresholds(0, 150, 10, 160, 20, 160)
         , m_visionTask(&m_camera,
                        IMAQ_IMAGE_RGB,
                        &m_colorThresholds,
                        2,      //two filter criteria
                        false,
                        m_filterCriteria,
                        ARRAYSIZE(m_filterCriteria))
         , m_frameRateFilter()
         , m_targetID(TARGET_MODE_LEFT_SIDE)
         , m_lastX(-1)
         , m_lastY(-1)
    {
        TLevel( INIT);
        TEnter();
        //
        // Initialize camera.
        //
        m_camera.WriteResolution(AxisCamera::kResolution_320x240);
        m_camera.WriteCompression(20);
        m_camera.WriteBrightness(10);
        //
        // Initialize filter criteria.
        //
        m_filterCriteria[0].parameter = IMAQ_MT_BOUNDING_RECT_WIDTH;
        m_filterCriteria[0].lower = 40;
        m_filterCriteria[0].upper = 400;
        m_filterCriteria[0].calibrated = false;
        m_filterCriteria[0].exclude = false;
        
        m_filterCriteria[1].parameter = IMAQ_MT_BOUNDING_RECT_HEIGHT;
        m_filterCriteria[1].lower = 30;
        m_filterCriteria[1].upper = 400;
        m_filterCriteria[1].calibrated = false;
        m_filterCriteria[1].exclude = false;
#if 0
        m_filterCriteria[2].parameter = IMAQ_MT_RATIO_OF_EQUIVALENT_RECT_SIDES;
        m_filterCriteria[2].lower = 1.0;
        m_filterCriteria[2].upper = 2.0;
        m_filterCriteria[2].calibrated = false;
        m_filterCriteria[2].exclude = false;
#endif

        RegisterTask(MOD_NAME, TASK_START_MODE | TASK_STOP_MODE);

        TExit();
    }   //VisionTarget

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~VisionTarget(
        void
        )
    {
        TLevel( INIT);
        TEnter();

        m_visionTask.SetTaskEnabled(false);
        UnregisterTask();

        TExit();
    }   //~VisionTarget

    /**
     * This function is called by the TaskMgr to start the task.
     * 
     * @param mode Specifies the calling mode (autonomous or teleop).
     */
    void
    TaskStartMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        m_visionTask.SetTaskEnabled(true);

        TExit();
    }   //TaskStartMode

    /**
     * This function is called by the TaskMgr to stop the task.
     * 
     * @param mode Specifies the calling mode (autonomous or teleop).
     */
    void
    TaskStopMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        m_visionTask.SetTaskEnabled(false);

        TExit();
    }   //TaskStopMode
    
    void
    SelectTarget(
        int targetID)
    {
        m_targetID = targetID;
        m_lastX = -1;
        m_lastY = -1;
    }

    /**
     * This function returns the target info with the given target ID.
     * 
     * @param targetID Specifies which target we want the info for.
     * @param targetInfo Points to the TARGETINFO structure to be filled in. 
     *
     * @return Returns true if target info is available, false otherwise.
     */
    bool
    GetTargetInfo(
        TARGETINFO *targetInfo
        )
    {
        bool fSuccess = false;
        vector<ParticleAnalysisReport> *particles;
        
        static int prevTime = 0;
        if(prevTime == 0)
        {
            prevTime = GetMsecTime();
        }

        TLevel(UTIL);
        TEnterMsg(("targetID=%d,info=%p", m_targetID, targetInfo));
        TSampling(("targetID=%d,info=%p", m_targetID, targetInfo));

        particles = m_visionTask.VisionGetTargets();
        if ((particles != NULL) && (particles->size() > 0))
        {
            //TInfo(("targeting took %5.3f milliseconds", m_frameRateFilter.FilterData(GetMsecTime() - prevTime)));
            prevTime = GetMsecTime();
            
            int particleIdx = -1;
            int recX;    //for differentiating side targets
            int recY;    //for differentiating top targets (when the perspective is off and we confuse a side with the top)
            int recClosestToCenter;      //when in CENTERMOST mode, keeps track of the lowest offset
            double recDist;
            //double recAspectRatio;      //when we see the side and the low
            
            for (unsigned i = 0; i < particles->size(); i++)
            {
                ParticleAnalysisReport *tParticle = &(particles->at(i));
                if(m_lastX == -1 && m_lastY == -1)      //select new target... otherwise pick the same target you got last time (closest to the spot where the last target was
                {
                    float aspectRatio = ((float) tParticle->boundingRect.width)/
                                        ((float) tParticle->boundingRect.height);
                    
                    switch(m_targetID)
                    {
                        case TARGET_MODE_TOP:
                            if (aspectRatio >= MIN_TOP_TARGET_ASPECT_RATIO &&
                                aspectRatio <= MAX_TOP_TARGET_ASPECT_RATIO)
                            {
    //MTS: Do you have the compare reversed??? tParticle->center_mass_y < recY???
                                if (particleIdx == -1 ||
                                    tParticle->center_mass_y > recY)
                                {
                                    particleIdx = i;
                                    recY = tParticle->center_mass_y;
                                }
                            }
                            break;
                            
                        case TARGET_MODE_LEFT_SIDE:
                            if (aspectRatio >= MIN_SIDE_TARGET_ASPECT_RATIO &&
                                aspectRatio <= MAX_SIDE_TARGET_ASPECT_RATIO)
                            {
                                if (particleIdx == -1 ||
                                    tParticle->center_mass_x < recX)
                                {
                                    particleIdx = i;
                                    recX = tParticle->center_mass_x;
                                }
                            }
                            break;
                            
                        case TARGET_MODE_RIGHT_SIDE:
                            if (aspectRatio >= MIN_SIDE_TARGET_ASPECT_RATIO &&
                                aspectRatio <= MAX_SIDE_TARGET_ASPECT_RATIO)
                            {
                                if (particleIdx == -1 ||
                                    (tParticle->center_mass_x > recX))
                                {
                                    particleIdx = i;
                                    recX = tParticle->center_mass_x;
                                    //recAspectRatio = aspectRatio;
                                }
                            }
                            break;
                            
                        case TARGET_MODE_CENTERMOST_TARGET:
                            int distToCenter = ABS(tParticle->center_mass_x -
                                                      SCREEN_CENTER_X);
                            if (particleIdx == -1 ||
                                distToCenter < recClosestToCenter)
                            {
                                if(aspectRatio <= MAX_ASPECT_RATIO_ANY_TARGET &&
                                   aspectRatio >= MIN_ASPECT_RATIO_ANY_TARGET)
                                {
                                    particleIdx = i;
                                    recClosestToCenter = distToCenter;
                                }
                            }
                            break;
                    }
                }
                else
                {
                    double distance = pow(tParticle->center_mass_x - m_lastX, 2) + 
                                      pow(tParticle->center_mass_y - m_lastY, 2);
                    if (particleIdx == -1 || 
                            distance <= recDist)
                    {
                        particleIdx = i;
                        recDist = distance;
                    }
                }
            }

            m_dashboardDataFormat->SendVisionData(particles,
                                                  (UINT8)particleIdx);

            if (particleIdx != -1)
            {
                m_lastX = particles->at(particleIdx).center_mass_x;
                m_lastY = particles->at(particleIdx).center_mass_y;
                //
                // Calculate target distance:
                // h1*d1 = h2*d2 = kd
                // => d2 = h1*d1/h2
                // => d2 = kd/h2
                //
                switch(m_targetID)
                {
                    case TARGET_MODE_TOP:
                        targetInfo->distance =
                            TOP_TARGET_DISTANCE_CONSTANT/
                            (float)particles->at(particleIdx).boundingRect.height;
                        
                        targetInfo->height = TARGET_HEIGHT_TOP;
                        break;

                    case TARGET_MODE_LEFT_SIDE:
                    case TARGET_MODE_RIGHT_SIDE:
                        targetInfo->distance =
                            SIDE_TARGET_DISTANCE_CONSTANT/
                            (float)particles->at(particleIdx).boundingRect.height;
                        
                        targetInfo->height = TARGET_HEIGHT_SIDE;
                        break;
                }

                //
                // Calculate target angle.
                // atan(w1/f) = asin(W1/d1)
                // => w1/f = tan(asin(W1/d1))
                // => f = w1/tan(asin(W1/d1))
                // angle = atan(w2/f)
                //
                targetInfo->hAngle = 
                    atan(((float)particles->at(particleIdx).center_mass_x - 
                                 SCREEN_CENTER_X)/
                         FOCAL_LENGTH);
                //
                // Convert angle from radians to degrees.
                //
                targetInfo->hAngle *= 180.0; //convert to degrees from radians
                targetInfo->hAngle /= PI;

                targetInfo->vAngle = 
                    atan(((SCREEN_CENTER_Y + GRAVITY_COMPENSATION_CONSTANT) - 
                            (float) particles->at(particleIdx).center_mass_y)/
                          FOCAL_LENGTH);
                targetInfo->vAngle *= 180.0;
                targetInfo->vAngle /= PI;
                
                fSuccess = true;
#ifdef _DEBUG_TARGET
                /*
                TInfo(("\n[%d]\tx=%3d, y=%3d, w=%3d, h=%3d\n",
                       particleIdx,
                       particles->at(particleIdx).center_mass_x,
                       particles->at(particleIdx).center_mass_y,
                       particles->at(particleIdx).boundingRect.width,
                       particles->at(particleIdx).boundingRect.height));
                TInfo(("\thAngle=%5.1f, dist=%5.1f, ht=%5.1f\n",
                       targetInfo->hAngle,
                       targetInfo->distance,
                       targetInfo->height));*/
                LCDPrintf((LCD_LINE2, "%d: angle=%5.1f",
                           particleIdx, targetInfo->hAngle));
                LCDPrintf((LCD_LINE3, "dist=%5.1f, ht=%5.1f",
                           targetInfo->distance, targetInfo->height));
                LCDPrintf((LCD_LINE4, "x=%3d, y=%3d",
                           particles->at(particleIdx).center_mass_x,
                           particles->at(particleIdx).center_mass_y));
                LCDPrintf((LCD_LINE5, "w=%3d, h=%3d",
                           particles->at(particleIdx).boundingRect.width,
                           particles->at(particleIdx).boundingRect.height));
                LCDPrintf((LCD_LINE3, "aRatio=%5.2f",
                           ((float) particles->at(particleIdx).boundingRect.width) / 
                           ((float) particles->at(particleIdx).boundingRect.height)));
#endif
            }
            else
            {
                TInfo(("No target found!"));
#ifdef _DEBUG_TARGET
                LCDPrintf((LCD_LINE6, "No matching target!"));
#endif
            }
        }
        else
        {
            TInfo(("No particles found!"));
#ifdef _DEBUG_TARGET
            LCDPrintf((LCD_LINE6, "No particles found!"));
#endif
        }

        SAFE_DELETE(particles);

        TExitMsg(("=%d", fSuccess));
        return fSuccess;
    }   //GetTargetInfo

}; //class VisionTarget
