typedef struct _targetInfo
{
    float distance;
    float hAngle;
    float vAngle;
    float vAngleAdj;
} TARGETINFO, *PTARGETINFO;

class VisionTarget
    : public CoopTask
{
private:
    PanTilt                *m_panTilt;
    Relay                   m_ringLightPower;
    DashboardDataFormat     m_dashData;
    AxisCamera             &m_camera;
    Threshold               m_colorThresholds;
    VisionTask              m_visionTask;
    ParticleFilterCriteria2 m_filterCriteria[3];
    bool                    m_fAutoTargetOn;

    void SetRingLightOn(bool fOn)
    {
        m_ringLightPower.Set(fOn? Relay::kOn: Relay::kOff);
    }   //SetRingLightOn

    bool GetTargetInfo(TARGETINFO *targetInfo)
    {
        bool fSuccess = false;
        vector<ParticleAnalysisReport> *particles;

        particles = m_visionTask.VisionGetTargets();
        if ((particles != NULL) && (particles->size() > 0))
        {
            ParticleAnalysisReport *target;

#ifdef _DEBUG_VISION
            for (unsigned i = 0; i < particles->size(); i++)
            {
                target = &(particles->at(i));
                printf("[%d] center(%d,%d), size(%d,%d)\n",
                       i,
                       target->center_mass_x,
                       target->center_mass_y,
                       target->boundingRect.width,
                       target->boundingRect.height);
            }
#endif
            m_dashData.SendVisionData(particles, 0);
            //
            // Calculate distance, hAngle, vAngle of the target.
            //
            target = &(particles->at(0));

            targetInfo->distance = TARGET_DISTANCE_SCALE/
                                   target->boundingRect.height;

            targetInfo->hAngle =
                RADIANS_TO_DEGREES(atan((float)(target->center_mass_x -
                                                SCREEN_CENTER_X)/
                                        FOCAL_LENGTH));

            targetInfo->vAngle =
                RADIANS_TO_DEGREES(atan((float)(SCREEN_CENTER_Y -
                                                target->center_mass_y)/
                                        FOCAL_LENGTH));
            //
            // t = distance/Vx0;
            // yDrop = (g*t*t)/2
            //
#if 0
            float dt = targetInfo->distance/DART_VX0;
            float y = (GRAVITY_CONSTANT_INCHES*dt*dt)/2.0;
            targetInfo->vAngleAdj = RADIANS_TO_DEGREES(
                                        atan(y/targetInfo->distance));
#endif
            targetInfo->vAngleAdj = 3.0;
            LCDPrintf((LCD_LINE2, "D=%3.0f,H=%5.1f,V=%5.1f",
                       targetInfo->distance,
                       targetInfo->hAngle,
                       targetInfo->vAngle + targetInfo->vAngleAdj));
#ifdef _DEBUG_VISION
            printf("Target: dist=%5.1f, hAngle=%5.1f, vAngle=%5.1f,vAdj=%5.2f\n",
                   targetInfo->distance,
                   targetInfo->hAngle,
                   targetInfo->vAngle,
                   targetInfo->vAngleAdj);
#endif

            fSuccess = true;
        }

        SAFE_DELETE(particles);
        return fSuccess;
    }   //GetTargetInfo

public:
    void SetAutoTarget(bool fOn)
    {
        m_fAutoTargetOn = fOn;
    }   //SetAutoTarget

    VisionTarget(PanTilt *panTilt)
        : m_panTilt(panTilt)
        , m_ringLightPower(RELAY_RINGLIGHT_POWER, Relay::kForwardOnly)
        , m_dashData()
        , m_camera(AxisCamera::GetInstance(CAMERA_IP))
        , m_colorThresholds(0, 150, 10, 160, 20, 160)
        , m_visionTask(&m_camera,
                       IMAQ_IMAGE_RGB,
                       &m_colorThresholds,
                       2,
                       false,
                       m_filterCriteria,
                       ARRAYSIZE(m_filterCriteria))
        , m_fAutoTargetOn(false)
    {
        //
        // Make sure ring light is off.
        //
        SetRingLightOn(false);
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
        m_filterCriteria[1].lower = 40;
        m_filterCriteria[1].upper = 400;
        m_filterCriteria[1].calibrated = false;
        m_filterCriteria[1].exclude = false;

        m_filterCriteria[2].parameter = IMAQ_MT_RATIO_OF_EQUIVALENT_RECT_SIDES;
        m_filterCriteria[2].lower = 3.0/4.0;
        m_filterCriteria[2].upper = 4.0/3.0;
        m_filterCriteria[2].calibrated = false;
        m_filterCriteria[2].exclude = false;

        RegisterTask("VisionTarget",
                     TASK_START_MODE | TASK_STOP_MODE | TASK_POST_PERIODIC);
    }   //VisionTarget

    virtual ~VisionTarget(void)
    {
        UnregisterTask();
        m_visionTask.SetTaskEnabled(false);
        SetRingLightOn(false);
    }   //~VisionTarget

    void TaskStartMode(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            SetRingLightOn(true);
            m_visionTask.SetTaskEnabled(true);
        }
    }   //TaskStopMode

    void TaskStopMode(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            m_visionTask.SetTaskEnabled(false);
            SetRingLightOn(false);
        }
    }   //TaskStopMode

    void TaskPostPeriodic(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            TARGETINFO targetInfo;
            if (GetTargetInfo(&targetInfo) && m_fAutoTargetOn)
            {
                m_panTilt->SetPanTiltTarget(
                        targetInfo.hAngle,
                        targetInfo.vAngle + targetInfo.vAngleAdj,
                        true);
            }
        }
    }   //TaskPostPeriodic
};  //class VisionTarget
