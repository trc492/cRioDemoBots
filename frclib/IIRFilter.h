#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="IIRFilter.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     IIRFilter class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _IIRFILTER_H
#define _IIRFILTER_H

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_FILTER
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "IIR"

/**
 * This module defines and implements the IIRFilter object.
 */
class IIRFilter
{
private:
    double  m_filteredValue;

public:
    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    IIRFilter(
        void
        ): m_filteredValue(0.0)
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //IIRFilter

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~IIRFilter(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~IIRFilter

    /**
     * This function applies the IIR filter to the data point. Essentially,
     * it averages the current data point with past data points and giving
     * a percentage weight between past value and current value. The lower
     * the weight of the current data point, the larger the filtering effect.
     * If the weight is 1.0, there is no filtering.
     * 
     * @param data Specifies the current data point.
     * @param weight Specifies the weight of the current data point (between
     *        0.0 to 1.0).
     *
     * @return Returns the filtered value.
     */
    double
    FilterData(
        double data,
        double weight
        )
    {
        TLevel(API);
        TEnterMsg(("data=%f,weight=%f", data, weight));

        m_filteredValue = m_filteredValue*(1 - weight) + data*weight;

        TExitMsg(("=%f", m_filteredValue));
        return m_filteredValue;
    }   //FilterData

};  //class IIRFilter

#endif  //ifndef _IIRFILTER_H
