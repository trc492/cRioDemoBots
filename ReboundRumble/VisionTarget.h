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
    float angle;
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
    Relay               m_ringLightPower;
    Threshold           m_colorThresholds;
    VisionTask          m_visionTask;
    ParticleFilterCriteria2 m_filterCriteria[2];

public:
    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    VisionTarget(
        DashboardDataFormat *dashboardDataFormat
        ): m_dashboardDataFormat(dashboardDataFormat)
         , m_camera(AxisCamera::GetInstance(CAMERA_IP))
         , m_ringLightPower(RELAY_RINGLIGHT_POWER, Relay::kForwardOnly)
        //
        // Good green values: RGB(0/100, 80/140, 40/140)
        // Obtained from camera snapshot in mspaint.
        //
         , m_colorThresholds(0, 100, 80, 140, 40, 140)
         , m_visionTask(&m_camera,
                        IMAQ_IMAGE_RGB,
                        &m_colorThresholds,
                        2,
                        false,
                        m_filterCriteria,
                        ARRAYSIZE(m_filterCriteria))
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
        m_ringLightPower.Set(Relay::kOff);
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

        m_ringLightPower.Set(Relay::kOn);
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
        m_ringLightPower.Set(Relay::kOff);

        TExit();
    }   //TaskStopMode

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
        int targetID,
        TARGETINFO *targetInfo
        )
    {
        bool fSuccess = false;
        vector<ParticleAnalysisReport> *particles;

        TLevel(API);
        TEnterMsg(("targetID=%d,info=%p", targetID, targetInfo));

        particles = m_visionTask.VisionGetTargets();
        if ((particles != NULL) && (particles->size() > 0))
        {
            int particleIdx = -1;
            double recX;
            double recY;

            for (unsigned i = 0; i < particles->size(); i++)
            {
                ParticleAnalysisReport *tParticle = &(particles->at(i));
                float aspectRatio = tParticle->boundingRect.width/
                                    tParticle->boundingRect.height; 

                if (aspectRatio >= MINIMUM_TARGET_ASPECT_RATIO && 
                    aspectRatio <= MAXIMUM_TARGET_ASPECT_RATIO)
                {
                    if (particleIdx == -1)
                    {
                        particleIdx = i;
                        recX = tParticle->center_mass_x;
                        recY = tParticle->center_mass_y;
                    }
                    else
                    {
                        switch (targetID)
                        {
                            case TARGET_TOP:
                                if (tParticle->center_mass_y < recY)
                                {
                                    recY = tParticle->center_mass_y;
                                    particleIdx = i;
                                }
                                break;

                            case TARGET_BOTTOM:
                                if (tParticle->center_mass_y > recY)
                                {
                                    recY = tParticle->center_mass_y;
                                    particleIdx = i;
                                }
                                break;

                            case TARGET_LEFT:
                                if (tParticle->center_mass_x < recX)
                                {
                                    recX = tParticle->center_mass_x;
                                    particleIdx = i;
                                }
                                break;

                            case TARGET_RIGHT:
                                if (tParticle->center_mass_x > recX)
                                {
                                    recX = tParticle->center_mass_x;
                                    particleIdx = i;
                                }
                                break;
                        }
                    }
                }
            }

            m_dashboardDataFormat->SendVisionData(particles,
                                                  (UINT8)particleIdx);

            if (particleIdx != -1)
            {
                //
                // Calculate target distance:
                // h1*d1 = h2*d2 = kd
                // => d2 = h1*d1/h2
                // => d2 = kd/h2
                //
                targetInfo->distance =
                    TARGET_DISTANCE_CONSTANT/
                    (float)particles->at(particleIdx).boundingRect.height;

                //
                // Calculate target height. 
                //
                double currKdh =
                         targetInfo->distance*
                         ((double)particles->at(particleIdx).center_mass_y -
                          SCREEN_CENTER_Y);
                float difLow = fabs(currKdh - TARGET_KDH_LOW);
                float difMid = fabs(currKdh - TARGET_KDH_MID);
                float difHigh = fabs(currKdh - TARGET_KDH_HIGH);

                if (difLow < difMid && difLow < difHigh)
                {
                    targetInfo->height = TARGET_HEIGHT_LOW;
                }
                else if (difMid < difLow && difMid < difHigh)
                {
                    targetInfo->height = TARGET_HEIGHT_MID;
                }
                else    // if (difHigh < difLow && difHigh < difMid)
                {
                    targetInfo->height = TARGET_HEIGHT_HIGH;
                }

                //
                // Calculate target angle.
                // atan(w1/f) = asin(W1/d1)
                // => w1/f = tan(asin(W1/d1))
                // => f = w1/tan(asin(W1/d1))
                // angle = atan(w2/f)
                //
                targetInfo->angle = 
                    atan((float)(particles->at(particleIdx).center_mass_x - 
                                 SCREEN_CENTER_X)/
                         FOCAL_LENGTH);
                //
                // Convert angle from radians to degrees.
                //
                targetInfo->angle *= 180.0; //convert to degrees from radians
                targetInfo->angle /= PI;
                fSuccess = true;
#ifdef _DEBUG_TARGET
                TInfo(("\n[%d]\tx=%3d, y=%3d, w=%3d, h=%3d\n",
                       particleIdx,
                       particles->at(particleIdx).center_mass_x,
                       particles->at(particleIdx).center_mass_y,
                       particles->at(particleIdx).boundingRect.width,
                       particles->at(particleIdx).boundingRect.height));
                TInfo(("\tangle=%5.1f, dist=%5.1f, ht=%5.1f\n",
                       targetInfo->angle,
                       targetInfo->distance,
                       targetInfo->height));
                LCDPrintf((LCD_LINE2, "%d: angle=%5.1f",
                           particleIdx, targetInfo->angle));
                LCDPrintf((LCD_LINE3, "dist=%5.1f, ht=%5.1f",
                           targetInfo->distance, targetInfo->height));
                LCDPrintf((LCD_LINE4, "x=%3d, y=%3d",
                           particles->at(particleIdx).center_mass_x,
                           particles->at(particleIdx).center_mass_y));
                LCDPrintf((LCD_LINE5, "w=%3d, h=%3d",
                           particles->at(particleIdx).boundingRect.width,
                           particles->at(particleIdx).boundingRect.height));
                LCDPrintf((LCD_LINE6, "Kdh=%5.1f", currKdh));
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
