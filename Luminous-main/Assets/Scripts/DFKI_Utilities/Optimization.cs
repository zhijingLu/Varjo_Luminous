using UnityEngine;
using System;
using System.Collections.Generic;

namespace DFKI_Utilities
{
    public class Optimization
    {
        public class ParameterHistory
        {
            public int capacity;
            public int size;
            private int lastId = 0;
            private List<float[]> p;
            private bool[] valid;

            public ParameterHistory(int _capacity, int _size)
            {
                if (_capacity < 2)
                    throw new ArgumentException("The parameter history must have a capacity >= 2", "_capacity");
                if (_size <= 0)
                    throw new ArgumentException("The parameter history must have a parameter size > 0", "_size");

                capacity = _capacity;
                size = _size;

                p = new List<float[]>(capacity);
                valid = new bool[capacity];
                for (var i = 0; i < capacity; i++)
                {
                    p.Add(new float[size]);
                    valid[i] = false;
                }
            }

            public void Store(ref float[] values)
            {
                if (values.Length != size)
                    throw new ArgumentException("The provided values have the wrong size", "values");

                lastId = (lastId + 1) % capacity;
                System.Array.Copy(values, p[lastId], size);
                valid[lastId] = true;
            }

            public void Clear()
            {
                for (var i = 0; i < capacity; i++)
                    valid[i] = false;
            }

            public float[] GetLast()
            {
                if (valid[lastId])
                    return p[lastId];
                return null;
            }

            public float[] GetOneBeforeLast()
            {
                int id = (lastId - 1) % capacity;
                if (id < 0)
                    id = capacity - 1;

                if (valid[id])
                    return p[id];
                return null;
            }
        }

        public enum Method
        {
            GradientDescent,            // fastest gradient descent
            GradientDescent_BLS,        // gradient descent with backtracking-line-search standard
            GradientDescent_BLS_Fast,      // gradient descent with backtracking-line-search faster
            GradientDescent_BLS_Single, // gradient descent with backtracking-line-search using single step size
        };

        [System.Serializable]
        public class Settings
        {
            private int n = 0; // number of parameters
            public bool debugMode = false;
            public int maxIter = 2000;
            public int minIter = 5;
            public float[] initSteps;
            public float[] maxSteps;
            public float Epsilon = 1e-7f;
            public float weightAcceleration = 0.0f;
            public float stepReductionBeta = 0.8f;
            public float stepIncreaseBeta = 1.2f; // Notice: do not use 1/stepReductionBeta, with defaults: 1.25, as this may cause jumping behavior during optimization

            public Settings(int _n)
            {
                // Default settings
                n = _n;

                initSteps = new float[n];
                maxSteps = new float[n];

                for (int i = 0; i < n; i++)
                {
                    initSteps[i] = 1.0f;
                    maxSteps[i] = 1.0f;
                }
            }

            public Settings(Settings other)
            {
                // deep copy of everything
                n = other.n;
                debugMode = other.debugMode;
                maxIter = other.maxIter;
                minIter = other.minIter;

                initSteps = new float[other.initSteps.Length];
                Array.Copy(other.initSteps, initSteps, initSteps.Length);
                maxSteps = new float[other.maxSteps.Length];
                Array.Copy(other.maxSteps, maxSteps, maxSteps.Length);

                Epsilon = other.Epsilon;
                weightAcceleration = other.weightAcceleration;
                stepIncreaseBeta = other.stepIncreaseBeta;
                stepReductionBeta = other.stepReductionBeta;
            }

            public int GetParameterSize()
            {
                return n;
            }

            public void SetMaxStep(float maxStep)
            {
                if (maxStep < 0)
                    throw new ArgumentException(String.Format("Error: the provided max step {0} is invalid", maxStep));

                SetMaxStep(0, n, maxStep);
            }

            public void SetMaxStep(int _from, int _to, float maxStep)
            {
                if (_from < 0 || _from >= n || _to < 0 || _to <= _from || _to > n)
                    throw new ArgumentException(String.Format("Error: the provided interval [{0},{1}] is outside the parameter's boundary {2}", _from, _to, n));
                if (maxStep < 0)
                    throw new ArgumentException(String.Format("Error: the provided max step {0} is invalid", maxStep));

                for (int i = _from; i < _to; i++)
                    maxSteps[i] = maxStep;
            }

            public void SetInitStep(float initStep)
            {
                if (initStep < 0)
                    throw new ArgumentException(String.Format("Error: the provided initial step {0} is invalid", initStep));

                SetInitStep(0, n, initStep);
            }

            public void SetInitStep(int _from, int _to, float initStep)
            {
                if (_from < 0 || _from >= n || _to < 0 || _to <= _from || _to > n)
                    throw new ArgumentException(String.Format("Error: the provided interval [{0},{1}] is outside the parameter's boundary {2}", _from, _to, n));
                if (initStep < 0)
                    throw new ArgumentException(String.Format("Error: the provided initial step {0} is invalid", initStep));

                for (int i = _from; i < _to; i++)
                    initSteps[i] = initStep;
            }

            public void SetMaxMinIter(int _minIter, int _maxIter)
            {
                if (_maxIter <= 0 || _minIter < 0 || _minIter >= _maxIter)
                    throw new ArgumentException(String.Format("Error: the provided interval [{0},{1}] is invalid", _minIter, _maxIter));

                minIter = _minIter;
                maxIter = _maxIter;
            }

            public void SetEpsilon(float _Epsilon)
            {
                Epsilon = _Epsilon;
            }

            public void SetStepReductionBeta(float _beta)
            {
                if (_beta <= 0)
                    throw new ArgumentException(String.Format("Error: the provided beta step reduction {0} is invalid", _beta));

                stepReductionBeta = _beta;
                float eps = (1.0f / _beta) * 0.4f;
                stepIncreaseBeta = (1.0f / _beta) - eps;
            }

            public void SetStepChangeBeta(float _betaReduction, float _betaIncrease)
            {
                if (_betaReduction <= 0)
                    throw new ArgumentException(String.Format("Error: the provided beta step reduction {0} is invalid", _betaReduction));
                if (_betaIncrease <= 0 || _betaIncrease < _betaReduction)
                    throw new ArgumentException(String.Format("Error: the provided beta step increase {0} is invalid", _betaIncrease));
                if (_betaIncrease >= 1.0f / _betaReduction)
                    Debug.LogWarning("Warning: the chosen beta step increase {0} may cause jumping behaviour during optimization");

                stepReductionBeta = _betaReduction;
                stepIncreaseBeta = _betaIncrease;
            }
        }

        // Delegate functions
        public delegate A DelegateGetEnergy<out T, in K, out A>(ref float[] T, bool K);
        public DelegateGetEnergy<float[], bool, float> GetEnergy;
        public Func<float[], bool, bool> SetParameters;

        // Main parameter list, history and optimization settings
        private float[] parameters;
        private ParameterHistory parametersHistory;
        private Settings settings;

        public Optimization(Settings _settings, DelegateGetEnergy<float[], bool, float> _GetEnergy, Func<float[], bool, bool> _SetParameters)
        {
            settings = _settings;
            GetEnergy = _GetEnergy;
            SetParameters = _SetParameters;

            parameters = new float[settings.GetParameterSize()];
            ResetParameters();

            parametersHistory = new ParameterHistory(2, settings.GetParameterSize());
        }

        public void SetInitParameters(float[] init)
        {
            if (init.Length != parameters.Length)
                throw new ArgumentException(String.Format("Error: the provided init parameters size {0} is different from the internal parameters size {1}", init.Length, parameters.Length));

            Array.Copy(init, parameters, init.Length);
            SetParameters(parameters, true);
        }

        private static void DebugPrintStep(int iter, int max_iter, float energy,
            ref float[] parameters, ref float[] gradient, ref float[] step, float diff)
        {
            string text = "";

            text += "Iter " + iter + "/" + max_iter + ") energy=" + energy + ", energy_delta=" + diff;
            for (var i = 0; i < parameters.Length; i++)
                text += "\n" + i + ") " + parameters[i] + " := \t\tprev + \t\tgradient (" + gradient[i] + ") * \t\tstep (" + step[i] + ")";

            Debug.Log(text);
        }

        public float Optimize(Method method)
        {
            switch (method)
            {
                case Method.GradientDescent:
                    return Optimize_GD();
                case Method.GradientDescent_BLS:
                    return Optimize_GD_BLS();
                case Method.GradientDescent_BLS_Fast:
                    return Optimize_GD_BLS_Fast();
                case Method.GradientDescent_BLS_Single:
                    return Optimize_GD_BLS_Single();
                default:
                    Debug.LogError("Error: The chosen optimization method is not supported");
                    return 0.0f;
            }
        }

        public void StoreInHistory()
        {
            // stores the current parameters in the parametersPrev
            parametersHistory.Store(ref parameters);
        }

        public bool AccelerateParameters()
        {
            // initializes the current parameters based on the previous parameters
            float[] prev = parametersHistory.GetLast();
            float[] prevprev = parametersHistory.GetOneBeforeLast();
            if (prev != null && prevprev != null)
            {
                for (var i = 0; i < parameters.Length; i++)
                    // parameters[i] = 2.0f * prevprev[i] - prev[i]; // acceleration
                    parameters[i] = prev[i] + 0.5f * (prev[i] - prevprev[i]); // linear interpolation

                // set and update all remaining variables depending on the parameters
                SetParameters(parameters, true);

                return true;
            }

            return false; // not enough available previous parameters
        }

        private float mod(float a, float b)
        {
            return (a % b + b) % b;
        }

        public float getEnergyAcceleration(ref float[] gradient, bool compute_gradient = true)
        {
            float energy = 0.0f;
            float mul = settings.weightAcceleration / parameters.Length;

            float[] prev = parametersHistory.GetLast();
            float[] prevprev = parametersHistory.GetOneBeforeLast();

            if (prev == null || prevprev == null || mul == 0.0f)
            {
                //Debug.Log("Acceleration Energy: not enough previous parameters OR acceleration weight is 0!");
                return mul * 1.0f;
            }

            for (var i = 0; i < parameters.Length; i++)
            {
                // angular distance
                float dist = parameters[i] - (2.0f * prev[i] - prevprev[i]);
                dist = mod(dist + 180.0f, 360.0f) - 180.0f; // account for angular periodicity (value in [0,180])]

                // compute error
                float distd = dist / 180.0f; // value in [0,1]
                float pow4 = Mathf.Pow(1.0f - distd, 4.0f);
                float mul4 = (4.0f * distd + 1);
                float e_i = pow4 * mul4; // wendland

                energy += mul * e_i;

                if (e_i < 1.0f && compute_gradient)
                {
                    // compute gradient
                    float deriv_pow4 = 4.0f * (1.0f - distd) * (-1.0f / 180.0f);
                    float deriv_mul4 = 4.0f / 180.0f;
                    float deriv_e_i = deriv_pow4 * mul4 + pow4 * deriv_mul4;

                    gradient[i] += mul * deriv_e_i;
                }
            }

            return energy;
        }

        private bool TestParametersDelta(float[] delta)
        {
            // testing setting of parameters delta without updating the current parameters!
            float[] newParameters = new float[parameters.Length];

            for (var p = 0; p < parameters.Length; p++)
                newParameters[p] = parameters[p] + delta[p];

            return SetParameters(newParameters, false);
        }

        public bool SetParametersDelta(float[] delta, bool compute_gradient = true)
        {
            for (var p = 0; p < parameters.Length; p++)
                parameters[p] = parameters[p] + delta[p];

            return SetParameters(parameters, compute_gradient);
        }

        public bool SetParametersSubset(uint[] id, float[] value)
        {
            // Consistency checks

            if (id == null || value == null || id.Length != value.Length)
                return false;

            for (int i = 0; i < id.Length; ++i)
            {
                if (id[i] >= parameters.Length)
                    return false;
            }

            // Do the actual thing
            for (int i = 0; i < id.Length; ++i)
                parameters[id[i]] = value[i];

            return SetParameters(parameters, true);
        }

        public bool ResetParameters()
        {
            Array.Clear(parameters, 0, parameters.Length);

            return SetParameters(parameters, true);
        }

        private float Optimize_GD_BLS_Fast()
        {
            // initializations
            int iter = -1;
            float prev_energy, energy = 0.0f, energy_step = 0.0f, diff = 0.0f;
            float[] gradient = new float[parameters.Length];
            float[] step = new float[parameters.Length];
            float[] step_prev = new float[parameters.Length];
            Array.Copy(settings.maxSteps, step_prev, parameters.Length); // Notice: initialized with the maximum steps!

            do
            {
                iter += 1;

                // get current energy and gradient vector with respect to each parameter
                prev_energy = energy;
                energy = GetEnergy(ref gradient, true);

                // backtracking-line-search to find suitable step size and update the parameters
                //Step(ref step, ref step_prev, ref gradient, energy);
                energy_step = 0.0f;
                Array.Clear(step, 0, step.Length);

                for (int p = 0; p < parameters.Length; p++)
                {
                    // Check previous step
                    // Notice: the next brackets code is a speed-up: remove this for some gain in accuracy!
                    if (-settings.Epsilon < step_prev[p] && step_prev[p] < settings.Epsilon)
                    {
                        step[p] = 0.0f; // focus on the remaining degrees of freedom (for speed)
                        continue;
                    }

                    // set current parameter's step to maximum
                    step[p] = gradient[p] * settings.maxSteps[p];

                    do
                    {
                        // compute energy with this step
                        TestParametersDelta(step);
                        energy_step = GetEnergy(ref gradient, false); // Notice: the gradient will not be updated!

                        if (energy > energy_step)
                            step[p] = step[p] * settings.stepReductionBeta; // further reduce the current step

                    } while (energy > energy_step && Mathf.Abs(step[p]) > settings.Epsilon);
                }

                // remember the previous step
                Array.Copy(step, step_prev, step.Length);

                // set new parameters delta
                SetParametersDelta(step, true);

                diff = Mathf.Abs(energy - prev_energy);

                if (settings.debugMode)
                {
                    DebugPrintStep(iter, settings.maxIter, energy, ref parameters, ref gradient, ref step, diff);
                }
            }
            while (iter < settings.maxIter && (iter < settings.minIter || diff > settings.Epsilon));

            // return the latest energy after optimization
            return energy;
        }

        private float Optimize_GD_BLS()
        {
            // initializations
            int iter = -1;
            float prev_energy, energy = 0.0f, energy_step = 0.0f, diff = 0.0f;
            float[] gradient = new float[parameters.Length];
            float[] step = new float[parameters.Length];

            do
            {
                iter += 1;

                // get current energy and gradient vector with respect to each parameter
                prev_energy = energy;
                energy = GetEnergy(ref gradient, true);

                // backtracking-line-search to find suitable step size and update the parameters
                energy_step = 0.0f;
                Array.Clear(step, 0, step.Length);

                for (int p = 0; p < parameters.Length; p++)
                {
                    // set current parameter's step to maximum
                    step[p] = gradient[p] * settings.maxSteps[p];

                    if (step[p] == 0.0f)
                        continue;

                    do
                    {
                        // compute energy with this step
                        TestParametersDelta(step);
                        energy_step = GetEnergy(ref gradient, false); // Notice: the gradient will not be updated!

                        if (energy > energy_step)
                            step[p] = step[p] * settings.stepReductionBeta; // further reduce the current step

                    } while (energy > energy_step && Mathf.Abs(step[p]) > settings.Epsilon);
                }

                // set new parameters delta
                SetParametersDelta(step, true);

                diff = Mathf.Abs(energy - prev_energy);

                if (settings.debugMode)
                {
                    DebugPrintStep(iter, settings.maxIter, energy, ref parameters, ref gradient, ref step, diff);
                }
            }
            while (iter < settings.maxIter && (iter < settings.minIter || diff > settings.Epsilon));

            // return the latest energy after optimization
            return energy;
        }

        private float Optimize_GD_BLS_Single()
        {
            // initializations
            int iter = -1;
            float prev_energy, energy = 0.0f, energy_step = 0.0f, diff = 0.0f, sum = float.MaxValue;
            float[] gradient = new float[parameters.Length];
            float[] step = new float[parameters.Length];

            do
            {
                iter += 1;

                // get current energy and gradient vector with respect to each parameter
                prev_energy = energy;
                energy = GetEnergy(ref gradient, true);

                // backtracking-line-search to find suitable step size and update the parameters
                for (int p = 0; p < parameters.Length; p++)
                {
                    // set current parameter's step to maximum
                    step[p] = gradient[p] * settings.maxSteps[p];
                }

                do
                {
                    // compute energy with this step
                    TestParametersDelta(step);
                    energy_step = GetEnergy(ref gradient, false); // Notice: the gradient will not be updated!

                    if (energy > energy_step)
                    {
                        sum = 0.0f;
                        for (int p = 0; p < parameters.Length; p++)
                        {
                            step[p] = step[p] * settings.stepReductionBeta; // further reduce the current step
                            sum += step[p];
                        }
                    }

                } while (energy > energy_step && sum > settings.Epsilon);

                // set new parameters delta
                SetParametersDelta(step, true);

                diff = Mathf.Abs(energy - prev_energy);

                if (settings.debugMode)
                {
                    DebugPrintStep(iter, settings.maxIter, energy, ref parameters, ref gradient, ref step, diff);
                }
            }
            while (iter < settings.maxIter && (iter < settings.minIter || diff > settings.Epsilon));

            // return the latest energy after optimization
            return energy;
        }

        private float Optimize_GD()
        {
            // initializations
            int iter = -1;
            float prev_energy, energy = float.MinValue, diff = 0.0f;
            float[] prev_gradient = new float[parameters.Length];
            float[] gradient = new float[parameters.Length];
            float[] delta = new float[parameters.Length];
            float[] step = new float[parameters.Length];
            Array.Copy(settings.initSteps, step, step.Length); // Notice: Assuming step remains always positive!

            do
            {
                iter += 1;

                // get current energy and gradient
                prev_energy = energy;
                Array.Copy(gradient, prev_gradient, gradient.Length);
                energy = GetEnergy(ref gradient, true);

                // update step size according to gradient evolution
                for (var i = 0; i < parameters.Length; i++)
                {
                    if (Mathf.Sign(gradient[i] * prev_gradient[i]) > 0)
                        step[i] = Mathf.Min(step[i] * settings.stepIncreaseBeta, settings.maxSteps[i]);
                    else
                        step[i] = step[i] * settings.stepReductionBeta;

                    // Notice: further dump the step size if the size of the gradient has changed consistently
                    //if (10.0f * Mathf.Abs(gradient[i]) < Mathf.Abs(prev_gradient[i]))
                    //if (Mathf.Abs(gradient[i]) - Mathf.Abs(prev_gradient[i]) >= 2.0f) // going from -1 to 1 and viceversa
                    //    step[i] = step[i] * 0.01f;

                    delta[i] = step[i] * gradient[i];
                }

                // set new parameters delta
                SetParametersDelta(delta, true);

                diff = Mathf.Abs(energy - prev_energy);

                if (settings.debugMode)
                {
                    // Debug
                    DebugPrintStep(iter, settings.maxIter, energy, ref parameters, ref gradient, ref step, diff);
                }
            }
            while (iter < settings.maxIter && (iter < settings.minIter || diff > settings.Epsilon));

            // return the latest energy after optimization
            return energy;
        }

        public float[] GetParameters()
        {
            // returns a copy of the parameters
            float[] copy = new float[parameters.Length];
            Array.Copy(parameters, copy, parameters.Length);
            return copy;
        }
    }

}