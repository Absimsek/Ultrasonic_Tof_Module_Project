#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <algorithm> // For sort
#include <iomanip>   // For formatting
#include <fstream>   // For CSV output
#include <cstdlib>   // For rand() and srand()
#include <ctime>     // For time() - REQUIRED FOR RANDOM SEED

using namespace std;

// ==========================================
// 1. PHYSICS LAYER: Acoustic Propagation
// ==========================================
// Physics II: Speed of sound approx 343 m/s at 20C
const double SPEED_OF_SOUND_M_S = 343.0;

// Requirement: Use POINTERS for the Physics conversion layer
// Input:  Pointer to raw Time-of-Flight (seconds)
// Output: Pointer to computed Distance (meters)
void convertTOFtoDistance(const double* timeArray, double* distArray, int n) {
    for(int i = 0; i < n; i++) {
        // Physics Eq: d = (v_sound * t) / 2
        // Pointer arithmetic: *(arr + i) is equivalent to arr[i]
        double t = *(timeArray + i);
        *(distArray + i) = (SPEED_OF_SOUND_M_S * t) / 2.0;
    }
}

// ==========================================
// 2. CALCULUS LAYER: Filtering & Derivation
// ==========================================

// A. Difference Equation (EMA Filter)
// Eq: S[n] = alpha * Y[n] + (1-alpha) * S[n-1]
class EMAFilter {
private:
    double alpha;
    double s_prev;
    bool isInitialized; // Flag to fix start-up lag

public:
    EMAFilter(double a) : alpha(a), s_prev(0.0), isInitialized(false) {}

    double apply(double y_n) {
        // If this is the very first reading, snap directly to it.
        if (!isInitialized) {
            s_prev = y_n;
            isInitialized = true;
            return y_n;
        }

        double s_n = alpha * y_n + (1.0 - alpha) * s_prev;
        s_prev = s_n;
        return s_n;
    }
};

// B. Statistical Filter (Median) - For Outlier Rejection
class MedianFilter {
private:
    vector<double> window;
    int size;

public:
    MedianFilter(int n) : size(n) {}

    double apply(double val) {
        window.push_back(val);
        // Maintain sliding window size
        if (window.size() > size) window.erase(window.begin());

        vector<double> sorted = window;
        sort(sorted.begin(), sorted.end());

        int len = sorted.size();
        if (len == 0) return 0.0;
        
        // Return median
        if (len % 2 == 1) return sorted[len / 2];
        else return (sorted[len / 2 - 1] + sorted[len / 2]) / 2.0;
    }
};

// C. Finite Difference (Velocity)
// Calculus Eq: v(t) = dx/dt approx (x[n] - x[n-1]) / dt
class Differentiator {
private:
    double x_prev;
    double dt;
    bool firstRun;

public:
    Differentiator(double timeStep) : dt(timeStep), x_prev(0), firstRun(true) {}

    double calculate(double x_n) {
        if(firstRun) {
            x_prev = x_n;
            firstRun = false;
            return 0.0;
        }
        double v = (x_n - x_prev) / dt;
        x_prev = x_n;
        return v;
    }
};

// ==========================================
// 3. CONTROL LAYER: PID (FIXED)
// ==========================================
class PID {
private:
    double Kp, Ki, Kd;
    double integral;
    double prevError;
    bool firstRun; // Added to fix Derivative Kick

public:
    PID(double p, double i, double d) 
        : Kp(p), Ki(i), Kd(d), integral(0), prevError(0), firstRun(true) {}

    double compute(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        
        // FIX: Handle First Run
        // If this is the first step, assume previous error = current error.
        // This makes derivative (error - prevError) = 0.
        if (firstRun) {
            prevError = error;
            firstRun = false;
        }

        // Riemann Sum for Integral
        integral += error * dt; 
        
        // Finite Difference for Derivative
        double derivative = (error - prevError) / dt;
        prevError = error;

        return Kp * error + Ki * integral + Kd * derivative;
    }
};

// ==========================================
// 4. MAIN PROJECT
// ==========================================
int main() {
    // --- NEW: Seed the random generator with current time ---
    // This ensures the noise pattern is different every time you run the code.
    srand(static_cast<unsigned int>(time(0))); 

    // Config
    const int NUM_STEPS = 200;
    const double DT = 0.05; // 50ms sampling
    const double NOISE_STD_SEC = 0.00001; // Noise in seconds
    
    // Data Storage: Using MAPS for organized access
    map<string, vector<double>> data;
    
    // Resize vectors
    data["Time"].resize(NUM_STEPS);
    data["TOF_Raw"].resize(NUM_STEPS);      
    data["Dist_Raw"].resize(NUM_STEPS);     
    data["Dist_Median"].resize(NUM_STEPS);
    data["Dist_EMA"].resize(NUM_STEPS);     
    data["Velocity"].resize(NUM_STEPS);
    data["PID_Out"].resize(NUM_STEPS);

    // Objects
    // EMA self-initializes now
    EMAFilter ema(0.1);       
    MedianFilter median(5);        
    Differentiator velocityCalc(DT);
    PID pid(1.5, 0.05, 0.2);

    cout << "=== ULTRASONIC SENSOR PROJECT (FINAL VERSION) ===\n";
    cout << "Physics: Acoustic TOF | Calculus: EMA & Finite Diff | C++: STL & Pointers\n\n";

    // --- STEP 1: GENERATE PHYSICS DATA ---
    for (int i = 0; i < NUM_STEPS; i++) {
        double t = i * DT;
        data["Time"][i] = t;

        // True Scenario: Static -> Ramp (Moving) -> Static
        double trueDist = 0.5; 
        if (t > 3.0 && t < 7.0) trueDist = 0.5 + (t - 3.0) * 0.2; // Moving at 0.2 m/s
        else if (t >= 7.0) trueDist = 1.3;

        // Reverse Physics: Calculate Time-of-Flight
        double trueTOF = (trueDist * 2.0) / SPEED_OF_SOUND_M_S;

        // Add Noise & Spikes
        double noise = ((rand() % 1000) / 1000.0 - 0.5) * NOISE_STD_SEC;
        if (rand() % 100 < 5) noise += 0.002; // 2ms Spike (5% chance)

        data["TOF_Raw"][i] = trueTOF + noise;
    }

    // --- STEP 2: APPLY PHYSICS CONVERSION (POINTERS) ---
    // Passing raw memory addresses to the function
    convertTOFtoDistance(data["TOF_Raw"].data(), data["Dist_Raw"].data(), NUM_STEPS);

    // --- STEP 3: PROCESSING LOOP ---
    for (int i = 0; i < NUM_STEPS; i++) {
        double rawD = data["Dist_Raw"][i];

        // A. Apply Median (Remove Spikes)
        double medD = median.apply(rawD);
        data["Dist_Median"][i] = medD;

        // B. Apply EMA (Smooth Noise)
        double emaD = ema.apply(medD);
        data["Dist_EMA"][i] = emaD;

        // C. Calculate Velocity (Calculus)
        data["Velocity"][i] = velocityCalc.calculate(emaD);

        // D. PID Control (Target = 1.0m)
        data["PID_Out"][i] = pid.compute(1.0, emaD, DT);
    }

    // --- STEP 4: OUTPUT RESULTS ---
    cout << fixed << setprecision(3);
    cout << "Time(s) | Raw(m) | EMA(m) | Vel(m/s) | PID_Out\n";
    cout << "----------------------------------------------\n";
    
    for (int i = 0; i < NUM_STEPS; i+=10) { 
        cout << setw(7) << data["Time"][i] << " | "
             << setw(6) << data["Dist_Raw"][i] << " | "
             << setw(6) << data["Dist_EMA"][i] << " | "
             << setw(8) << data["Velocity"][i] << " | "
             << setw(7) << data["PID_Out"][i] << endl;
    }

    // Export to CSV
    ofstream file("project_results.csv");
    file << "Time,Raw,Median,EMA,Velocity,PID\n";
    for(int i=0; i<NUM_STEPS; i++) {
        file << data["Time"][i] << "," << data["Dist_Raw"][i] << "," 
             << data["Dist_Median"][i] << "," << data["Dist_EMA"][i] << ","
             << data["Velocity"][i] << "," << data["PID_Out"][i] << "\n";
    }
    file.close();
    cout << "\nData exported to 'project_results.csv'" << endl;

    cout << "\nAnalysis saved to 'calculus_project_data.csv'.\n";
    return 0;
}