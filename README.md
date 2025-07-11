# 🛸 PID Drone Height Controller

This project demonstrates a **PID-based motor controller** for a simulated drone system using a potentiometer as a feedback sensor. It uses real-time height estimation based on trigonometric calculations and LED indicators to manage motor power and status feedback.

---

## 🚀 Features

* Real-time drone height estimation from potentiometer angle
* Motor speed control using PID algorithm (Proportional, Integral, Derivative)
* LED status indicators:

  * 🟩 Green: Target height achieved
  * 🟨 Yellow: Below target
  * 🟥 Red: Above target
* Safety auto-shutdown after maintaining target height for 5 seconds
* Serial monitoring for black-box logging and telemetry

---

## 🔧 Hardware Requirements

* Arduino Uno or compatible board
* Potentiometer (for simulating drone angle)
* L298N motor driver module
* DC motor
* LEDs (Red, Yellow, Green)
* Resistors (220–330Ω)
* Breadboard and jumper wires

---

## 📐 Key Control Concepts

* **Height Estimation**: `height = 0.491 × sin(potentiometer_angle)`
* **Target Height**: 0.43 m
* **Tolerance**: ±2 cm
* **PID Control Logic**:

  ```
  motor_speed = hover_power + (KP × error + KI × integral + KD × derivative)
  ```

---

## 📄 Files

```
pid-drone-height-controller/
├── pid_height_controller.ino         # Main Arduino sketch
├── serial_output_sample.txt          # Sample serial log
├── wiring_diagram.png (optional)     # Add your circuit diagram
└── README.md                         # This file
```

---

## 📈 Sample Serial Output

> See full: [`serial_output_sample.txt`](serial_output_sample.txt)

```
Time,      Power,    Angle,    Height,    Error
0.00,      100,      -0.0,      0.000,     -0.430
0.04,      112,      -2.5,      0.021,     -0.409
...
5.00,      120,      -55.0,     0.430,      0.000

4. Shuting down...
Time to max height: 5.00 s
Corrected height: 0.430 m
Corrected error: 0.000 m
```

---

## 🧪 Tuning Parameters

* `KP = 150`
* `KI = 2`
* `KD = 80`
* `hover_power = 100` (base motor PWM duty cycle)

---

## 👨‍💻 Author

**Nehemiah  Kimutai**
GitHub: [@Nemick](https://github.com/Nemick)

---

## 📌 Future Improvements

* Integrate actual ultrasonic height sensor (e.g., HC-SR04 or ToF)
* Add OLED screen display of live telemetry
* Enable live PID tuning via serial input or Bluetooth
* Log data to SD card for offline analysis

---

## 📜 License

This project is intended for educational and experimental use.
