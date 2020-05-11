import java.time.Duration;

public class PidController
{
    private double processVariable = 0;

    public PidController(double GainProportional, double GainIntegral, double GainDerivative, double OutputMax, double OutputMin)
    {
        this.GainDerivative = GainDerivative;
        this.GainIntegral = GainIntegral;
        this.GainProportional = GainProportional;
        this.OutputMax = OutputMax;
        this.OutputMin = OutputMin;
    }

    /// <summary>
    /// The controller output
    /// </summary>
    /// <param name="timeSinceLastUpdate">timespan of the elapsed time
    /// since the previous time that ControlVariable was called</param>
    /// <returns>Value of the variable that needs to be controlled</returns>
    public double ControlVariable(Duration timeSinceLastUpdate)
    {
        double error = SetPoint - getProcessVariable();

        // integral term calculation
        IntegralTerm += (GainIntegral * error * timeSinceLastUpdate.getSeconds());
        IntegralTerm = Clamp(IntegralTerm);

        // derivative term calculation
        double dInput = processVariable - ProcessVariableLast;
        //double derivativeTerm = GainDerivative * (dInput / timeSinceLastUpdate.getSeconds());

        // proportional term calcullation
        double proportionalTerm = GainProportional * error;

        double output = proportionalTerm;//+ IntegralTerm ;- derivativeTerm;

        output = Clamp(output);

        return output;
    }

    /// <summary>
    /// The derivative term is proportional to the rate of
    /// change of the error
    /// </summary>
    public double GainDerivative = 0;

    /// <summary>
    /// The integral term is proportional to both the magnitude
    /// of the error and the duration of the error
    /// </summary>
    public double GainIntegral  = 0;

    /// <summary>
    /// The proportional term produces an output value that
    /// is proportional to the current error value
    /// </summary>
    /// <remarks>
    /// Tuning theory and industrial practice indicate that the
    /// proportional term should contribute the bulk of the output change.
    /// </remarks>
    public double GainProportional = 0;

    /// <summary>
    /// The max output value the control device can accept.
    /// </summary>
    public double OutputMax  = 0;

    /// <summary>
    /// The minimum ouput value the control device can accept.
    /// </summary>
    public double OutputMin = 0;

    /// <summary>
    /// Adjustment made by considering the accumulated error over time
    /// </summary>
    /// <remarks>
    /// An alternative formulation of the integral action, is the
    /// proportional-summation-difference used in discrete-time systems
    /// </remarks>
    public double IntegralTerm = 0;


    /// <summary>
    /// The current value
    /// </summary>

    public double getProcessVariable() {
        return processVariable;
    }

    // Setter
    public void setProcessVariable(double value) {
        ProcessVariableLast = this.processVariable;
        processVariable = value;
    }

    /// <summary>
    /// The last reported value (used to calculate the rate of change)
    /// </summary>
    public double ProcessVariableLast = 0;

    /// <summary>
    /// The desired value
    /// </summary>
    public double SetPoint = 0;

    /// <summary>
    /// Limit a variable to the set OutputMax and OutputMin properties
    /// </summary>
    /// <returns>
    /// A value that is between the OutputMax and OutputMin properties
    /// </returns>
    /// <remarks>
    /// Inspiration from http://stackoverflow.com/questions/3176602/how-to-force-a-number-to-be-in-a-range-in-c
    /// </remarks>
    private double Clamp(double variableToClamp)
    {
        if (variableToClamp <= OutputMin) { return OutputMin; }
        if (variableToClamp >= OutputMax) { return OutputMax; }
        return variableToClamp;
    }
}