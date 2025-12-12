#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>

using namespace std;

//
// =========================
// 1. EMA FILTER CLASS
// =========================
class EMAFilter {
private:
    double alpha;
    double Ys;

public:
    EMAFilter(double factor, double initialValue)//
        : alpha(factor), Ys(initialValue) {}

    double apply(double Xn) {
        double Yn = alpha * Xn + (1.0 - alpha) * Ys;
        Ys = Yn;
        return Yn;
    }
};

//
// =========================
// 2. MEDIAN FILTER CLASS
// =========================
class MedianFilter {
private:
    double* window;
    int size;
    int count;
    int current;

public:
    MedianFilter(int windowSize) : size(windowSize), count(0), current(0) {
        window = new double[size];
        for (int i = 0; i < size; i++) window[i] = 0.0;
    }

    ~MedianFilter() {// ???????????????????????????
        delete[] window;
    }

    void bubbleSort(double* arr, int n) {
        for (int i = 0; i < n - 1; i++) {
            for (int j = 0; j < n - i - 1; j++) {
                if (arr[j] > arr[j + 1]) {
                    double temp = arr[j];
                    arr[j] = arr[j + 1];
                    arr[j + 1] = temp;
                }
            }
        }
    }

    double apply(double Xn) {
        window[current] = Xn;
        current = (current + 1) % size;
        if (count < size) count++;

        double* temp = new double[count];
        for (int i = 0; i < count; i++) {
            temp[i] = window[i];
        }

        bubbleSort(temp, count);

        double result;
        if (count % 2 == 1) {
            result = temp[count / 2];
        }
        else {
            result = (temp[count / 2 - 1] + temp[count / 2]) / 2.0;
        }

        delete[] temp;
        return result;
    }
};

//
// =========================
// 3. PID CONTROLLER CLASS
// =========================
class PID {
private:
    double Kp, Ki, Kd;
    double integral;
    double prevError;

public:
    PID(double p, double i, double d) : Kp(p), Ki(i), Kd(d), integral(0), prevError(0) {}

    double compute(double setpoint, double measurement) {
        double error = setpoint - measurement;
        integral += error;
        double derivative = error - prevError;
        prevError = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }
};

//
// =========================
// 4. RMSE Function
// =========================
double calculateRMSE(double* actual, double* filtered, int N) {
    double sum = 0;
    for (int i = 0; i < N; i++) {
        double e = actual[i] - filtered[i];
        sum += e * e;
    }
    return sqrt(sum / N);
}

//
// =========================
// 5. ASCII GRAPH FUNCTION
// =========================
void drawAsciiGraph(double value, double scale = 1.0) {
    int length = (int)(value / scale);
    if (length < 0) length = 0;
    if (length > 80) length = 80;

    for (int i = 0; i < length; i++)
        cout << "#";
    cout << " (" << value << ")";
}

//
// =========================
// 6. GAUSSIAN NOISE FUNCTION
// =========================
double gaussianNoise(double mean, double stddev) {
    double u1 = (double)rand() / RAND_MAX;
    double u2 = (double)rand() / RAND_MAX;

    // Box-Muller transform - M_PI yerine 3.14159265358979323846 kullan
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14159265358979323846 * u2);

    return mean + z0 * stddev;
}

//
// =========================
// 7. MAIN PROJECT
// =========================
int main() {
    srand((unsigned int)time(0)); // Warning düzeltildi

    const int NUM_STEPS = 200;
    const double GAUSS_STD = 1.5;
    const double SPIKE_P = 0.05;
    const double SPIKE_VALUE = 250.0;

    const double ALPHA_SLOW = 0.2;
    const double ALPHA_FAST = 0.7;
    const int MEDIAN_WINDOW = 7;

    // Filter objects
    EMAFilter emaSlow(ALPHA_SLOW, 50.0);
    EMAFilter emaFast(ALPHA_FAST, 50.0);
    MedianFilter median(MEDIAN_WINDOW);
    PID pid(0.4, 0.01, 0.15);

    // Arrays for data storage
    double* actual = new double[NUM_STEPS];
    double* raw = new double[NUM_STEPS];
    double* medF = new double[NUM_STEPS];
    double* emaS = new double[NUM_STEPS];
    double* emaF = new double[NUM_STEPS];
    double* pidOut = new double[NUM_STEPS];

    double distance = 50.0;

    cout << "\n=== REAL-TIME SENSOR + FILTER + PID SIMULATION ===\n";

    for (int i = 0; i < NUM_STEPS; i++) {
        // Generate test scenario
        if (i < 60) distance = 50;
        else if (i < 160) distance = 50 + (i - 60) * 0.5;
        else distance = 100;

        // Generate noisy reading with spikes
        // Main döngüsünün içindeki "Generate noisy reading" kısmını bununla değiştir:

// Ses hızı (cm/mikrosaniye cinsinden, yakl. 343 m/s)
        const double SPEED_OF_SOUND = 0.0343;

        // 1. ADIM: FİZİK (Time-of-Flight Hesaplaması)
        // Sensör gidiş-dönüş süresini ölçer (t = 2 * d / v)
        double trueTimeMicroseconds = (distance * 2.0) / SPEED_OF_SOUND;

        // 2. ADIM: GÜRÜLTÜYÜ ZAMANA EKLE (Noise Characterization)
        // Gürültü aslında zamanlamada olur (jitter, yankı gecikmesi vb.)
        double noisyTime = trueTimeMicroseconds + gaussianNoise(0.0, GAUSS_STD * 10.0); // Standart sapmayı zaman ölçeğine uydur

        // Spike (Ani Sıçrama) Simülasyonu (Zaman hatası olarak)
        if ((double)rand() / RAND_MAX < SPIKE_P) {
            noisyTime += (SPIKE_VALUE * 2.0) / SPEED_OF_SOUND; // Mesafeyi zamana çevirip ekledik
        }

        // 3. ADIM: ÖLÇÜLEN MESAFEYE DÖNÜŞ (Simulated Sensor Reading)
        // d = (t * v) / 2
        double r = (noisyTime * SPEED_OF_SOUND) / 2.0;

        // Negatif mesafe kontrolü (Fiziksel olarak imkansız)
        if (r < 0) r = 0;

        // Store data
        actual[i] = distance;
        raw[i] = r;

        // Apply filters
        medF[i] = median.apply(r);
        emaS[i] = emaSlow.apply(r);
        emaF[i] = emaFast.apply(r);

        // PID control
        pidOut[i] = pid.compute(80, emaS[i]);

        // --- Terminal Real-time Output ---
        cout << "\nSTEP " << i << "\n";
        cout << "Actual: " << distance << "\n";
        cout << "Raw   : " << r << "\n";
        cout << "Median: " << medF[i] << "\n";
        cout << "EMA_s : " << emaS[i] << "\n";
        cout << "EMA_f : " << emaF[i] << "\n";
        cout << "PID   : " << pidOut[i] << "\n";

        // --- ASCII graph ---
        cout << "GRAPH RAW    : "; drawAsciiGraph(r, 3.0);
        cout << "\nGRAPH MEDIAN : "; drawAsciiGraph(medF[i], 3.0);
        cout << "\nGRAPH EMA_S  : "; drawAsciiGraph(emaS[i], 3.0);
        cout << "\n-------------------------------------------\n";
    }

    //
    // === CSV OUTPUT ===
    //
    ofstream f("output.csv");
    f << "Step,Actual,Raw,Median,EMA_Slow,EMA_Fast,PID\n";
    for (int i = 0; i < NUM_STEPS; i++) {
        f << i << "," << actual[i] << "," << raw[i] << ","
            << medF[i] << "," << emaS[i] << "," << emaF[i] << "," << pidOut[i] << "\n";
    }
    f.close();

    //
    // === RMSE ===
    //
    cout << "\n=== RMSE RESULTS ===\n";
    cout << "Median Filter RMSE: " << calculateRMSE(actual, medF, NUM_STEPS) << "\n";
    cout << "EMA Slow RMSE     : " << calculateRMSE(actual, emaS, NUM_STEPS) << "\n";
    cout << "EMA Fast RMSE     : " << calculateRMSE(actual, emaF, NUM_STEPS) << "\n";
    cout << "\nCSV Output: output.csv\n";

    // Clean up memory
    delete[] actual;
    delete[] raw;
    delete[] medF;
    delete[] emaS;
    delete[] emaF;
    delete[] pidOut;

    return 0;
}