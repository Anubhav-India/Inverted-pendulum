Digital Twin of a Segway / Inverted Pendulum
An interactive, full-stack Digital Twin web application simulating an inverted pendulum (Segway) control system. This project bridges classic control theory with modern web development, featuring a real-time physics engine, state-feedback control, and a full-order Luenberger observer.

🚀 Features
Nonlinear Physics Simulation: Solves real-time differential equations governing a cart-pendulum system using SciPy.

State-Feedback Controller (
u=−Ks): Utilizes pole placement techniques to stabilize the inherently unstable upright equilibrium position.

Luenberger Observer (
L): Simulates state estimation under noisy sensor conditions, comparing true plant states against estimated states.

Interactive Dashboard: Modern dark-mode UI built with Tailwind CSS, featuring an HTML5 Canvas viewport and dynamic Chart.js telemetry graphs.

Disturbance Rejection: Includes interactive controls to apply impulse force disturbances and observe real-time recovery.

📚 Syllabus Mapping (Control Systems II)
This project maps directly to core advanced control engineering curricula:

Unit 1: Non-linear system modeling (cart-pendulum equations).

Unit 4: State-space representation, controllability, state feedback controller design via pole placement, and full-order observer design.

Unit 5: Linearization around equilibrium points and system stability analysis.

🛠️ Tech Stack
Backend: Python, FastAPI, NumPy, SciPy, python-control

Frontend: HTML5, Tailwind CSS, JavaScript (ES6+), Chart.js, HTML5 Canvas API

Deployment Ready: Configured for seamless deployment on Render (backend) and Vercel (frontend).

📁 Project Directory Structure
Plaintext
segway-digital-twin/
├── backend/
│   ├── main.py              # FastAPI server, CORS middleware, and simulation routes
│   ├── control_model.py     # Physics, linearization, K-gain, and L-observer math
│   └── requirements.txt     # Python dependencies
├── frontend/
│   ├── index.html           # Main dashboard UI
│   ├── js/
│   │   ├── app.js           # Canvas rendering loop & API integration
│   │   └── charts.js        # Chart.js telemetry graph configurations
│   └── README.md            # Project documentation
└── .gitignore
⚙️ Local Installation & Setup
1. Clone the Repository
Bash
git clone https://github.com/your-username/segway-digital-twin.git
cd segway-digital-twin
2. Set Up the Python Backend
Bash
cd backend

# Create and activate a virtual environment
python -m venv venv
# Windows:
venv\Scripts\activate
# macOS / Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Start the FastAPI server
uvicorn main:app --reload --host 127.0.0.1 --port 8000
The backend API will be live at [http://127.0.0.1:8000](http://127.0.0.1:8000). You can inspect endpoints via [http://127.0.0.1:8000/docs](http://127.0.0.1:8000/docs).

3. Run the Frontend UI
Open a new terminal window and navigate to the frontend directory:

Bash
cd frontend
Open index.html in your browser:

Recommended: Use the Live Server extension in VS Code.

Alternatively, open index.html directly in your web browser (ensure your API base URL in app.js points to [http://127.0.0.1:8000](http://127.0.0.1:8000)).

🌐 Deployment Instructions
Deploying Backend on Render
Push your repository to GitHub.

Create a new Web Service on Render.

Set the Root Directory to backend.

Use the following build/start commands:

Build Command: pip install -r requirements.txt

Start Command: uvicorn main:app --host 0.0.0.0 --port $PORT

Copy your live Render URL (e.g., [https://segway-backend.onrender.com](https://segway-backend.onrender.com)).

Deploying Frontend on Vercel
Update your frontend JavaScript API fetch path (app.js) to point to your live Render backend URL.

Import your repository into Vercel, setting the Root Directory to frontend.

Deploy instantly to get your public web application link!
