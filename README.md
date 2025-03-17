# **7983S Worlds Code** 🚀

### **By: Christian Mills**  

---  

## 📌 **Project Overview**  
This repository contains the advanced competition code for **VEX Team 7983S**, developed for the **VEX Worlds Championship**. The program includes **autonomous pathing, driver control enhancements, and real-time adaptability**, ensuring precise and efficient robot performance.

## 📂 **Project Structure**  
- **Main Code:** Located in `src/main`  
- **Pure Pursuit Files:** Found in the `static` folder  
- **Configuration Files & Autonomous Recording:** Managed dynamically during operation  

## 🚀 **Key Innovations**  
### **1️⃣ Advanced Odometry & Pathing**  
- Uses tracking wheels and an IMU for **precise positioning**.  
- Implements **Pure Pursuit** algorithms for smooth, arc-based autonomous movement.  

### **2️⃣ Dynamic & Adaptive Autonomous Control**  
- **Real-time sensor feedback** (e.g., `colorSensor`, `clampDistance`) adjusts robot actions on the fly.  
- **Multi-threaded execution** enables independent processes like auto-clamping and object sorting.  

### **3️⃣ Custom Motion Control & PID Optimization**  
- **Fine-tuned PID controllers** optimize drivetrain and mechanism performance.  
- **ExpoDriveCurves** enhance driver control for smoother acceleration and steering.  

### **4️⃣ On-the-Fly Autonomous Selection & Recording**  
- **LCD-based auton selector** allows selecting routines before a match.  
- **Driver input recording system** logs movements for performance review and replay.  

### **5️⃣ Modular & Expandable Control System**  
- **Custom `controlSetup` & `recordingSetup` structures** make motor and sensor handling efficient and flexible.  
- Designed for **easy expansion** as new strategies and mechanisms are developed.  

## ⚙️ **Setup & Installation**  
1. Clone this repository:  
   ```bash
   git clone https://github.com/Runtime-3rr0r/7983S-worlds.git
   ```  
2. Open in **VEXcode Pro V5** or a compatible C++ environment.  
3. Deploy to your **VEX V5 Robot Brain**.  
4. Use the LCD or controller to select an autonomous routine.  

## 🎮 **Usage Instructions**  
- **Autonomous Mode:** Runs based on the selected routine.  
- **Driver Control Mode:** Uses optimized steering and acceleration curves.  
- **Testing Mode:** `test_mode()` available for diagnostics and tuning.  

## 🤝 **Contributors**  
- **Albert Moon** – Driver/Design  
- **Bree Harris** – Notebook/Strategy  
- **Christian Mills** – Programming/Design  
- **John Eoh** – Notebook/Strategy  
- **Tyler Kipp** – Backup Driver/Builder  
- **Vance Stevens** – Builder/Design  

## 📜 **License**  
Licensed under the **MIT License**. See `LICENSE` for details.  

---  
🔥 *Innovating for victory, one algorithm at a time!*

