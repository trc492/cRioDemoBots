#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="VisionTask.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     VisionTask class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _VISIONTASK_H
#define _VISIONTASK_H

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_VISION
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Vision"

//#define _WRITE_IMAGES
//#define _DUMP_REPORTS

#define VTF_ENABLED             0x00000001
#define VTF_ONE_SHOT            0x00000002
#define VTF_DO_CONVEX_HULL      0x00000004

static
void
ProcessImageTask(
    void *vt
    );

/**
 * This module defines and implements the VisionTask object. The
 * VisionTask object consists of an IP camera. It takes a snapshot
 * from the camera and detects the target particles in the snapshot. It
 * returns a list of target objects.
 */
class VisionTask
{
private:
    AxisCamera              *m_camera;
    ImageType                m_imageType;
    Threshold               *m_colorThresholds;
    int                      m_sizeThreshold;
    ParticleFilterCriteria2 *m_filterCriteria;
    int                      m_numCriteria;
    float                    m_taskWaitPeriod;

    vector<ParticleAnalysisReport> *m_targets;
    UINT32                   m_vtFlags;
    Task                     m_task;
    SEM_ID                   m_semaphore;
    ColorImage               m_cameraImage;
    BinaryImage             *m_filteredImage;

public:

    /**
     * This function checks for a fresh image and search for targets.
     */
    void
    ProcessImage(
        void
        )
    {
        TLevel(FUNC);
        TEnter();

#ifdef _WRITE_IMAGES
        static bool fWroteImages = false;
#endif
        if (((m_vtFlags & (VTF_ENABLED | VTF_ONE_SHOT)) != 0) &&
            m_camera->IsFreshImage())
        {
            int err = ERR_SUCCESS;
            BinaryImage *image = NULL;
            BinaryImage *thresholdImage = NULL;
            BinaryImage *bigObjImage = NULL;
            BinaryImage *convexHullImage = NULL;
            BinaryImage *filteredImage = NULL;
#ifdef _VISION_PERF
            UINT32 totalTime = 0;
            UINT32 startTime;
            UINT32 deltaTime;
#endif

#ifdef _VISION_PERF
            startTime = GetMsecTime();
#endif
            m_camera->GetImage(&m_cameraImage);
#ifdef _VISION_PERF
            deltaTime = GetMsecTime() - startTime;
            totalTime += deltaTime;
            TInfo(("AcquireImageTime = %d", deltaTime));
#endif
            //
            // Filter the image by color.
            //
#ifdef _VISION_PERF
            startTime = GetMsecTime();
#endif
            switch (m_imageType)
            {
            case IMAQ_IMAGE_RGB:
                thresholdImage =
                    m_cameraImage.ThresholdRGB(*m_colorThresholds);
                break;

            case IMAQ_IMAGE_HSL:
                thresholdImage =
                    m_cameraImage.ThresholdHSL(*m_colorThresholds);
                break;

            default:
                TErr(("Unsupported image type (type=%d).", m_imageType));
                break;
            }
#ifdef _VISION_PERF
            deltaTime = GetMsecTime() - startTime;
            totalTime += deltaTime;
            TInfo(("ColorThresholdTime = %d", deltaTime));
#endif
            image = thresholdImage;

            if (thresholdImage == NULL)
            {
                err = imaqGetLastError();
                TErr(("Failed to filter image with thresholds (err=%d).",
                      err));
            }
            else if (m_sizeThreshold > 0)
            {
                //
                // Remove small objects
                //
#ifdef _VISION_PERF
                startTime = GetMsecTime();
#endif
                bigObjImage = image->RemoveSmallObjects(
                                false, m_sizeThreshold);
#ifdef _VISION_PERF
                deltaTime = GetMsecTime() - startTime;
                totalTime += deltaTime;
                TInfo(("BigObjFilterTime = %d", deltaTime));
#endif
                image = bigObjImage;

                if (bigObjImage == NULL)
                {
                    err = imaqGetLastError();
                    TErr(("Failed to filter image with size (err=%d).", err));
                }
            }

            if ((err == ERR_SUCCESS) && (m_vtFlags & VTF_DO_CONVEX_HULL))
            {
#ifdef _VISION_PERF
                startTime = GetMsecTime();
#endif
                convexHullImage = image->ConvexHull(false);
#ifdef _VISION_PERF
                deltaTime = GetMsecTime() - startTime;
                totalTime += deltaTime;
                TInfo(("ConvexHullTime = %d", deltaTime));
#endif
                image = convexHullImage;

                if (convexHullImage == NULL)
                {
                    err = imaqGetLastError();
                    TErr(("Failed to generate Convex Hull image (err=%d).",
                          err));
                }
            }

            if ((err == ERR_SUCCESS) && (m_filterCriteria != NULL))
            {
#ifdef _VISION_PERF
                startTime = GetMsecTime();
#endif
                filteredImage = image->ParticleFilter(m_filterCriteria,
                                                      m_numCriteria);
#ifdef _VISION_PERF
                deltaTime = GetMsecTime() - startTime;
                totalTime += deltaTime;
                TInfo(("ParticleFilterTime = %d", deltaTime));
#endif
                image = filteredImage;

                if (filteredImage == NULL)
                {
                    err = imaqGetLastError();
                    TErr(("Failed to filter image based on criteria (err=%d).",
                          err));
                }
            }

            if (err == ERR_SUCCESS)
            {
#ifdef _VISION_PERF
                startTime = GetMsecTime();
#endif
                vector<ParticleAnalysisReport> *reports =
                        image->GetOrderedParticleAnalysisReports();
#ifdef _VISION_PERF
                deltaTime = GetMsecTime() - startTime;
                totalTime += deltaTime;
                TInfo(("GetReportTime = %d", deltaTime));
#endif

                if (reports == NULL)
                {
                    err = imaqGetLastError();
                    TErr(("Failed to get particle analysis reports (err=%d).",
                          err));
                }
                else
                {
#ifdef _DUMP_REPORTS
                    TInfo(("NumParticles = %d", reports->size()));
#endif
#ifdef _WRITE_IMAGES
                    if (!fWroteImages)
                    {
                        m_cameraImage.Write("/colorImage.bmp");
                        if (thresholdImage != NULL)
                        {
                            thresholdImage->Write("/thresholdImage.bmp");
                        }
                        if (bigObjImage != NULL)
                        {
                            bigObjImage->Write("/bigObjImage.bmp");
                        }
                        if (convexHullImage != NULL)
                        {
                            convexHullImage->Write("/convexHullImage.bmp");
                        }
                        if (filteredImage != NULL)
                        {
                            filteredImage->Write("/filteredImage.bmp");
                        }
                        fWroteImages = true;
                    }
#endif
#ifdef _DUMP_REPORTS
                    for (unsigned i = 0; i < reports->size(); i++)
                    {
                        ParticleAnalysisReport *p = &(reports->at(i));
                        TInfo(("%d: (%d,%d) [%d,%d/%d,%d]: AR=%5.2f,Q=%5.2f,Per=%4.2f",
                               i,
                               p->center_mass_x,
                               p->center_mass_y,
                               p->boundingRect.left,
                               p->boundingRect.top,
                               p->boundingRect.width,
                               p->boundingRect.height,
                               (float)p->boundingRect.width/(float)p->boundingRect.height,
                               p->particleQuality,
                               p->particleToImagePercent));
                    }
#endif
                    CRITICAL_REGION(m_semaphore)
                    {
                        SAFE_DELETE(m_targets);
                        m_targets = reports;
                        reports = NULL;

                        SAFE_DELETE(m_filteredImage);
                        m_filteredImage = filteredImage;
                        filteredImage = NULL;

                        m_vtFlags &= ~VTF_ONE_SHOT;
                    }
                    END_REGION;
                }
            }
#ifdef _VISION_PERF
            TInfo(("Total Elapsed Time = %d", totalTime));
#endif
            SAFE_DELETE(filteredImage);
            SAFE_DELETE(convexHullImage);
            SAFE_DELETE(bigObjImage);
            SAFE_DELETE(thresholdImage);
        }
        
        TExit();
    }   //ProcessImage

    /**
     * This function checks if the vision task is enabled.
     *
     * @return Returns true if vision task is enabled, false otherwise.
     */
    bool
    IsEnabled(
        void
        )
    {
        TLevel(API);
        TEnter();

        bool fEnabled = (m_vtFlags & VTF_ENABLED) != 0;

        TExitMsg(("=%x", fEnabled));
        return fEnabled;
    }   //IsEnabled

    /**
     * This function set vision task state to enabled or disabled.
     *
     * @param fEnable If true, enables the vision task, disables otherwise.
     */
    void
    SetTaskEnabled(
        bool fEnable
        )
    {
        TLevel(API);
        TEnterMsg(("fEnable=%x", fEnable));

        if (fEnable)
        {
            m_vtFlags |= VTF_ENABLED;
        }
        else
        {
            m_vtFlags &= ~VTF_ENABLED;
        }

        TExit();
    }   //SetTaskEnabled

    /**
     * This function gets vision task wait period. This is the wait period
     * between processing each image frame.
     *
     * @return Returns the current task wait period.
     */
    float
    GetTaskWaitPeriod(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExit();
        return m_taskWaitPeriod;
    }   //SetTaskWaitPeriod

    /**
     * This function sets vision task wait period. This is the wait period
     * between processing each image frame.
     *
     * @param taskWaitPeriod Specifies the task wait period in seconds.
     */
    void
    SetTaskWaitPeriod(
        float taskWaitPeriod
        )
    {
        TLevel(API);
        TEnterMsg(("waitPeriod=%f", taskWaitPeriod));

        m_taskWaitPeriod = taskWaitPeriod;

        TExit();
    }   //SetTaskWaitPeriod

    /**
     * Constructor for the class object.
     * Create instances of all the components.
     *
     * @param camera Specifies the camera object.
     * @param imageType Specifies the image type.
     * @param colorThresholds Specifies the color thresholds.
     * @param sizeThreshold Specifies the object size threshold.
     * @param fDoConvexHull Specifies whether to do a ConvexHull operation.
     * @param filterCriteria Points to the particle criteria array.
     * @param numCriteria Specifies the number of criteria in the array.
     * @param taskWaitPeriod Specifies the wait period between procesing each
     *        image frame.
     */
    VisionTask(
        AxisCamera *camera,
        ImageType imageType,
        Threshold *colorThresholds,
        int sizeThreshold = 0,
        bool fDoConvexHull = false,
        ParticleFilterCriteria2 *filterCriteria = NULL,
        int numCriteria = 0,
        float taskWaitPeriod = 0.05
        ): m_camera(camera)
         , m_imageType(imageType)
         , m_colorThresholds(colorThresholds)
         , m_sizeThreshold(sizeThreshold)
         , m_filterCriteria(filterCriteria)
         , m_numCriteria(numCriteria)
         , m_taskWaitPeriod(taskWaitPeriod)
         , m_targets(NULL)
         , m_vtFlags(0)
         , m_task("VisionTask", (FUNCPTR)ProcessImageTask)
         , m_semaphore(0)
         , m_cameraImage(imageType)
         , m_filteredImage(NULL)
    {
        TLevel(INIT);
        TEnterMsg(("camera=%p,type=%d,colorTh=%p,sizeTh=%d,fConvexHull=%d,criteria=%p,numCrit=%d,waitPeriod=%4.2f",
                    camera, imageType, colorThresholds, fDoConvexHull,
                    sizeThreshold, filterCriteria, numCriteria,
                    taskWaitPeriod));

        if (fDoConvexHull)
        {
            m_vtFlags |= VTF_DO_CONVEX_HULL;
        }

        m_semaphore = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
 
        if (!m_task.Start((INT32)this))
        {
            TErr(("Failed to start vision taget task."));
        }

        TExit();
    }   //VisionTask
 
    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~VisionTask(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        SetTaskEnabled(false);
        m_task.Stop();
        SAFE_DELETE(m_filteredImage);
        SAFE_DELETE(m_targets);
        semFlush(m_semaphore);

        TExit();
    }   //~VisionTask

    /**
     * This function returns the array of targets found.
     *
     * @param filteredImage Optionally points to a variable to receive
     *        the filtered image.
     *
     * @return Returns the array of targets found.
     */
    vector<ParticleAnalysisReport> *
    VisionGetTargets(
        BinaryImage **filteredImage = NULL
        )
    {
        vector<ParticleAnalysisReport> *targets;

        TLevel(API);
        TEnter();

        CRITICAL_REGION(m_semaphore)
        {
            if (((m_vtFlags & VTF_ENABLED) == 0) && (m_targets == NULL))
            {
                //
                // If vision task is not enabled and we don't have any targets,
                // let's just do this one shot.
                //
                m_vtFlags |= VTF_ONE_SHOT;
            }

            targets = m_targets;
            m_targets = NULL;
            if (filteredImage != NULL)
            {
                *filteredImage = m_filteredImage;
                m_filteredImage = NULL;
            }
        }
        END_REGION;

        TExitMsg(("=%p", targets));
        return targets;
    }   //VisionGetTargets

};  //class VisionTask

/**
 * This task continuously checks for a fresh image and search for targets.
 *
 * Do not call this function directly.
 */
static
void
ProcessImageTask(
    void *vt
    )
{
    TLevel(TASK);
    TEnter();

    while (true)
    {
        float waitPeriod = ((VisionTask*)vt)->GetTaskWaitPeriod();

        ((VisionTask*)vt)->ProcessImage();
        if (waitPeriod > 0.0)
        {
            Wait(waitPeriod);
        }
    }

    TExit();
}   //ProcessImageTask

#endif  //ifndef _VISIONTASK_H
