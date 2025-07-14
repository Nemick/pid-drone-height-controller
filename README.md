# PID Drone Height Controller🛸 

This project demonstrates a **PID-based motor controller** for a simulated drone system using a potentiometer as a feedback sensor. It uses real-time height estimation based on trigonometric calculations and LED indicators to manage motor power and status feedback.

---

## 🚀 Features

* Real-time drone height estimation from potentiometer angle
* Motor speed control using PID algorithm (Proportional, Integral, Derivative)
* LED status indicators:

  * 🟩 Green: Target height achieved
  * 🟨 Yellow: Below target
  * 🟥 Red: Above target
* Thrust direction indicators (🟦 Up, 🟧 Down)
* Safety auto-shutdown after maintaining target height for 5 seconds
* Serial monitoring for black-box logging and telemetry
* Dual L298N motor driver support for 4-motor setup

---

## 🔧 Hardware Requirements

* Arduino Uno or compatible board
* Potentiometer (for simulating drone pitch angle)
* 2 × L298N motor driver modules
* 4 × DC motors (simulated for thrust)
* LEDs (Red, Yellow, Green, Blue, Amber)
* Resistors (220–330Ω)
* Breadboard and jumper wires

---

## 📐 Key Control Concepts

* **Height Estimation**: `height = 0.491 × sin(potentiometer_angle)`
* **Target Height**: 0.39 m (adjustable)
* **Tolerance**: ±2 cm
* **PID Control Logic**:

  ```
  motor_speed = hover_power + (KP × error + KI × integral + KD × derivative)
  ```
* **Thrust logic**:

  * > 70% PWM if rising
  * <50% PWM if descending
  * \=50% PWM for hover

---

## 📁 Project Files

```
pid-drone-height-controller/
├── multi_motor_pid_height_controller.ino   # Main Arduino sketch
├── Simulation Video.mp4                    # Simulation demo
├── SERIAL MONITOR DATA.pdf                 # Sample serial output log
├── Flowchart.pdf                           # Project logic flowchart
├── Wokwi source file.zip                   # Virtual simulation environment
└── README.md                               # Project overview
```

---

## 📈 Sample Serial Output

> See full: \[`SERIAL MONITOR DATA.pdf`]\(SERIAL MONITOR DATA.pdf)

```
Time,      Power,    Angle,    Height,    Error
0.00,      100,      -0.0,      0.000,     -0.430
...
4. Shuting down...
Time taken to reach Maximum height: 5.00 s
Current Angle: -55.0°
First Height reached: 0.000 m
Height reached After correction: 0.430 m
Maximum error encountered: 0.430 m
Error after correction: 0.000 m
```

## 🎥 Simulation Video

Watch the live simulation demo:
👉 [Click here to view on Google Drive](https://drive.google.com/file/d/175ymC1blj4Ponb_DDwkFx8deSn1rvoea/view?usp=sharing)

---

## 🧪 Tuning Parameters

* `KP = 150`
* `KI = 2`
* `KD = 80`
* `hover_power = 128`
* `upThrust_power = 204`
* `downThrust_power = 77`

---

## 👨‍💻 Author

**Nehemiah Kimutai**
GitHub: [@Nemick](https://github.com/Nemick)

---

## 📌 Future Improvements

* Add actual ultrasonic or ToF height sensors
* OLED screen for real-time data
* PID parameter tuning via serial or Bluetooth
* Data logging to SD card for field analysis

---

## 📜 License

This project is intended for academic, simulation, and demonstration use.
