#include "DashboardDataFormat.h"
#include "RobotInfo.h"

DashboardDataFormat::DashboardDataFormat(void)
{
    analogModule1 = AnalogModule::GetInstance(1);
    analogModule2 = AnalogModule::GetInstance(2);
    digitalModule1 = DigitalModule::GetInstance(1);
    digitalModule2 = DigitalModule::GetInstance(2);
}

DashboardDataFormat::~DashboardDataFormat()
{
}

void DashboardDataFormat::SendVisionData(
        vector<ParticleAnalysisReport> *particles,
        UINT8 currIdx)
{
    Dashboard &dash =
            DriverStation::GetInstance()->GetHighPriorityDashboardPacker();
    
    //
    // Vision target Info.
    //
    dash.AddCluster();                  //Begin: Target Info
    {
        int targetsSent = 0;
        //
        // Loop through all particles, but stop after you send 4
        //
        for (unsigned i = 0; i < particles->size() && targetsSent < 4; i++)
        {
            ParticleAnalysisReport *p = &(particles->at(i));
            float aspectRatio = p->boundingRect.width/
                                p->boundingRect.height;

            if (aspectRatio >= MINIMUM_TARGET_ASPECT_RATIO && 
                aspectRatio <= MAXIMUM_TARGET_ASPECT_RATIO)
            {
                targetsSent++;
                dash.AddCluster();      //  Begin: Target Rect
                {
                    //
                    // The dashboard will select color based on this bool
                    //
                    dash.AddBoolean(i == currIdx);  
                    dash.AddU32(p->boundingRect.left);
                    dash.AddU32(p->boundingRect.top);
                    dash.AddU32(p->boundingRect.left +
                                p->boundingRect.width);
                    dash.AddU32(p->boundingRect.top +
                                p->boundingRect.height);
                }
                dash.FinalizeCluster(); //  End: Target Rect
            }
        }
        
        //
        // If we have less than 4 targets, we need to send null info for
        // the rest.
        //
        while (targetsSent < 4)
        {
            targetsSent++;
            dash.AddCluster();          //  Begin: Target Rect
            {
                dash.AddBoolean(false);
                dash.AddU32(0);
                dash.AddU32(0);
                dash.AddU32(0);
                dash.AddU32(0);
            }
            dash.FinalizeCluster();     //  End: Target Rect
        }
    }
    dash.FinalizeCluster();             //End: Target Info
    dash.Finalize();
}

void DashboardDataFormat::SendIOPortData()
{
    Dashboard &dash =
            DriverStation::GetInstance()->GetLowPriorityDashboardPacker();
    dash.AddCluster();
    {
        dash.AddCluster();
        { //analog modules 
            dash.AddCluster();
            {
                if (analogModule1 != NULL)
                {
                    for (int i = 1; i <= 8; i++)
                    {
                        dash.AddFloat(
                                (float) analogModule1->GetAverageVoltage(i));
                    }
                }
                else
                {
                    for (int i = 1; i <= 8; i++)
                    {
                        dash.AddFloat(0.0);
                    }
                }
            }
            dash.FinalizeCluster();
            dash.AddCluster();
            {
                if (analogModule2 != NULL)
                {
                    for (int i = 1; i <= 8; i++)
                    {
                        dash.AddFloat(
                                (float) analogModule2->GetAverageVoltage(i));
                    }
                }
                else
                {
                    for (int i = 1; i <= 8; i++)
                    {
                        dash.AddFloat(0.0);
                    }
                }
            }
            dash.FinalizeCluster();
        }
        dash.FinalizeCluster();

        dash.AddCluster();
        { //digital modules
            dash.AddCluster();
            {
                if (digitalModule1 != NULL)
                {
                    dash.AddU8(digitalModule1->GetRelayForward());
                    dash.AddU8(digitalModule1->GetRelayReverse());
                    dash.AddU16((short) digitalModule1->GetDIO());
                    dash.AddU16((short) digitalModule1->GetDIODirection());
                    dash.AddCluster();
                    {
                        for (int i = 1; i <= 10; i++)
                        {
                            dash.AddU8(
                                    (unsigned char) digitalModule1->GetPWM(
                                            i));
                        }
                    }
                    dash.FinalizeCluster();
                }
                else
                {
                    dash.AddU8(0);
                    dash.AddU8(0);
                    dash.AddU16(0);
                    dash.AddU16(0);
                    dash.AddCluster();
                    {
                        for (int i = 1; i <= 10; i++)
                            dash.AddU8(0);
                    }
                    dash.FinalizeCluster();
                }
            }
            dash.FinalizeCluster();

            dash.AddCluster();
            {
                if (digitalModule2 != NULL)
                {
                    dash.AddU8(digitalModule2->GetRelayForward());
                    dash.AddU8(digitalModule2->GetRelayForward());
                    dash.AddU16((short) digitalModule2->GetDIO());
                    dash.AddU16(digitalModule2->GetDIODirection());
                    dash.AddCluster();
                    {
                        for (int i = 1; i <= 10; i++)
                        {
                            dash.AddU8(
                                    (unsigned char) digitalModule2->GetPWM(
                                            i));
                        }
                    }
                    dash.FinalizeCluster();
                }
                else
                {
                    dash.AddU8(0);
                    dash.AddU8(0);
                    dash.AddU16(0);
                    dash.AddU16(0);
                    dash.AddCluster();
                    {
                        for (int i = 1; i <= 10; i++)
                            dash.AddU8(0);
                    }
                    dash.FinalizeCluster();
                }
            }
            dash.FinalizeCluster();
        }
        dash.FinalizeCluster();

        // Can't read solenoids without an instance of the object
        dash.AddU8((char) 0);
    }
    dash.FinalizeCluster();
    dash.Finalize();
}
