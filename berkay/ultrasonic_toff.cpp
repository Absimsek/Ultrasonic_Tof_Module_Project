#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <algorithm> // std::sort

using namespace std;

// ==========================================
// CALCULUS II: EMA (Difference Equation)
// ==========================================
// Teorik Temel: Birinci dereceden doğrusal fark denklemi (First-Order Linear Difference Equation)
// Formül: y[n] = α * x[n] + (1 - α) * y[n-1]
class EMAFilter {
private:
    double alpha;
    double y_prev; // y[n-1]

public:
    EMAFilter(double factor, double initialValue)
        : alpha(factor), y_prev(initialValue) {}

    double apply(double x_n) {
        // Fark denkleminin kodlanmış hali:
        double y_n = alpha * x_n + (1.0 - alpha) * y_prev;

        y_prev = y_n; // Bir sonraki adım için hafızaya al
        return y_n;
    }
};

// ==========================================
// PROGRAMMING: Median Filter
// ==========================================
// Amaç: Veri setindeki aykırı değerleri (Outliers) temizlemek.
// Yöntem: Sliding Window + Sorting
class MedianFilter {
private:
    vector<double> window;
    int size;
    int currentIdx;

public:
    MedianFilter(int windowSize) : size(windowSize), currentIdx(0) {
        window.resize(size, 0.0);
    }

    double apply(double x_n) {
        // Dairesel buffer (Programming Skill)
        window[currentIdx] = x_n;
        currentIdx = (currentIdx + 1) % size;

        // Sıralama için kopya oluştur
        vector<double> sortedWindow = window;
        sort(sortedWindow.begin(), sortedWindow.end());

        // Medyan bulma
        if (size % 2 == 1) return sortedWindow[size / 2];
        else return (sortedWindow[size / 2 - 1] + sortedWindow[size / 2]) / 2.0;
    }
};

// ==========================================
// CALCULUS II: Finite Difference (Velocity)
// ==========================================
// Teorik Temel: Türev (Derivative)
// v(t) = dx/dt ≈ (x[n] - x[n-1]) / Δt
class Differentiator {
private:
    double x_prev;
    double dt;

public:
    Differentiator(double timeStep, double initialValue)
        : dt(timeStep), x_prev(initialValue) {}

    double calculateVelocity(double x_n) {
        // Sonlu farklar yöntemi ile hız hesabı
        double velocity = (x_n - x_prev) / dt;
        x_prev = x_n;
        return velocity;
    }
};

// ==========================================
// PROGRAMMING: PID Controller
// ==========================================
class PID {
private:
    double Kp, Ki, Kd;
    double integral;
    double prevError;

public:
    PID(double p, double i, double d) : Kp(p), Ki(i), Kd(d), integral(0), prevError(0) {}

    double compute(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        integral += error * dt; // Riemann Sum (İntegral yaklaşımı)
        double derivative = (error - prevError) / dt; // Finite Difference (Türev yaklaşımı)
        prevError = error;

        return Kp * error + Ki * integral + Kd * derivative;
    }
};

// Yardımcı Fonksiyon: Gürültü Üretici
double generateNoise(double stddev) {
    double u1 = (double)rand() / RAND_MAX;
    double u2 = (double)rand() / RAND_MAX;
    if (u1 < 1e-10) u1 = 1e-10;
    double z = sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14159 * u2);
    return z * stddev;
}

// ==========================================
// MAIN PROGRAM
// ==========================================
int main() {
    srand(static_cast<unsigned int>(time(0)));

    // SİMÜLASYON PARAMETRELERİ
    const int NUM_STEPS = 200;
    const double DT = 0.1; // Zaman adımı (saniye) -> Calculus için önemli
    const double NOISE_STD = 2.0;

    // Nesneler (Programming)
    EMAFilter ema(0.2, 0.0);
    MedianFilter median(5);
    Differentiator velocityCalc(DT, 0.0); // Hız hesaplayıcı
    PID pid(0.5, 0.01, 0.1);

    // Veri Saklama (std::vector)
    vector<double> timeStamps(NUM_STEPS);
    vector<double> rawDist(NUM_STEPS);
    vector<double> filteredDist(NUM_STEPS);
    vector<double> velocity(NUM_STEPS);
    vector<double> pidOutput(NUM_STEPS);

    cout << "=== CALCULUS II & PROGRAMMING PROJECT ===\n";
    cout << "Focus: Difference Equations, Finite Differences, Algorithms\n\n";

    double currentDist = 0.0;
    double target = 50.0; // PID Hedefi

    // --- SİMÜLASYON DÖNGÜSÜ ---
    for (int i = 0; i < NUM_STEPS; i++) {
        double t = i * DT;
        timeStamps[i] = t;

        // 1. Hareket Senaryosu (Matematiksel Fonksiyon)
        // 0-10sn arası dur, sonra rampa yap (hız oluşsun), sonra dur.
        if (t < 5.0) currentDist = 20.0;
        else if (t < 15.0) currentDist = 20.0 + (t - 5.0) * 5.0; // Hız = 5 m/s
        else currentDist = 70.0;

        // 2. Gürültü Ekleme (Raw Data)
        double noise = generateNoise(NOISE_STD);
        // %10 ihtimalle büyük sapma (Spike) ekle - Median filtresini test etmek için
        if ((double)rand() / RAND_MAX < 0.1) noise += 20.0;

        double r = currentDist + noise;
        rawDist[i] = r;

        // 3. Filtreleme (EMA: Difference Equation Uygulaması)
        // Önce Median ile spike temizle, sonra EMA ile yumuşat
        double m = median.apply(r);
        double f = ema.apply(m);
        filteredDist[i] = f;

        // 4. CALCULUS: Hız Hesaplama (Finite Difference)
        // d(mesafe) / dt = hız
        // Not: Ham verinin türevini alırsak gürültü patlar. 
        // Filtrelenmiş verinin türevini alıyoruz (Calculus uygulamasının başarısı).
        velocity[i] = velocityCalc.calculateVelocity(f);

        // 5. PID Kontrol (Opsiyonel)
        pidOutput[i] = pid.compute(target, f, DT);

        // Terminal Çıktısı (Her 10 adımda bir özet)
        if (i % 10 == 0) {
            cout << "T=" << fixed << setprecision(1) << t << "s | ";
            cout << "Raw: " << setprecision(2) << r << " | ";
            cout << "Filt: " << f << " | ";
            cout << "Vel: " << velocity[i] << " unit/s" << endl;
        }
    }

    // --- CSV ÇIKTISI (Excel/Python Analizi İçin) ---
    ofstream file("calculus_project_data.csv");
    file << "Time,RawDistance,FilteredDistance,CalculatedVelocity,PID_Out\n";
    for (int i = 0; i < NUM_STEPS; i++) {
        file << timeStamps[i] << ","
            << rawDist[i] << ","
            << filteredDist[i] << ","
            << velocity[i] << ","
            << pidOutput[i] << "\n";
    }
    file.close();

    cout << "\nAnalysis saved to 'calculus_project_data.csv'.\n";
    return 0;
}