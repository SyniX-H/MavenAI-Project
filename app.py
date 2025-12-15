from flask import Flask, request, jsonify, render_template, send_from_directory
from flask_cors import CORS
from werkzeug.utils import secure_filename
import os
import json
from pathlib import Path
import sys

# Add backend modules to path
sys.path.insert(0, os.path.dirname(__file__))

# Import custom modules
from backend.checker.ros2_checker import ROS2CodeChecker
from backend.simulator.gazebo_runner1 import GazeboSimRunner

app = Flask(__name__)
CORS(app)

# Configuration
UPLOAD_FOLDER = Path('./uploads')
OUTPUT_FOLDER = Path('./simulation_output')
ALLOWED_EXTENSIONS = {'zip'}

UPLOAD_FOLDER.mkdir(exist_ok=True)
OUTPUT_FOLDER.mkdir(exist_ok=True)

app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['OUTPUT_FOLDER'] = OUTPUT_FOLDER
app.config['MAX_CONTENT_LENGTH'] = 50 * 1024 * 1024  # 50MB max


def allowed_file(filename):
    """Check if file extension is allowed"""
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


@app.route('/')
def index():
    """Serve the main page"""
    return render_template('index.html')


@app.route('/api/upload', methods=['POST'])
def upload_file():
    """Handle file upload"""
    try:
        # Check if file is in request
        if 'file' not in request.files:
            return jsonify({'error': 'No file part in request'}), 400
        
        file = request.files['file']
        
        # Check if user selected a file
        if file.filename == '':
            return jsonify({'error': 'No file selected'}), 400
        
        # Validate file type
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
            filepath = app.config['UPLOAD_FOLDER'] / filename
            file.save(str(filepath))
            
            return jsonify({
                'success': True,
                'filename': filename,
                'filepath': str(filepath)
            }), 200
        
        return jsonify({'error': 'Invalid file type. Only .zip files allowed'}), 400
    
    except Exception as e:
        return jsonify({'error': f'Upload failed: {str(e)}'}), 500


@app.route('/api/check', methods=['POST'])
def check_code():
    """Run code validation"""
    try:
        data = request.json
        filepath = data.get('filepath')
        
        if not filepath:
            return jsonify({'error': 'Filepath is required'}), 400
        
        if not os.path.exists(filepath):
            return jsonify({'error': 'File not found'}), 404
        
        # Run validation
        print(f"→ Validating package: {filepath}")
        checker = ROS2CodeChecker()
        text_report, json_report = checker.validate_package(filepath)
        
        # Save reports
        report_file = app.config['OUTPUT_FOLDER'] / 'validation_report.txt'
        with open(report_file, 'w') as f:
            f.write(text_report)
        
        json_file = app.config['OUTPUT_FOLDER'] / 'validation_report.json'
        with open(json_file, 'w') as f:
            json.dump(json_report, f, indent=2)
        
        print(f"✓ Validation complete: {json_report['status']}")
        
        return jsonify({
            'success': True,
            'text_report': text_report,
            'json_report': json_report
        }), 200
        
    except Exception as e:
        print(f"✗ Validation error: {str(e)}")
        import traceback
        traceback.print_exc()
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/simulate', methods=['POST'])
def run_simulation():
    """Run Gazebo simulation"""
    try:
        data = request.json
        package_path = data.get('package_path')
        duration = data.get('duration', 30)
        
        if not package_path:
            return jsonify({'error': 'Package path is required'}), 400
        
        # Validate duration
        if not isinstance(duration, int) or duration < 10 or duration > 300:
            return jsonify({'error': 'Duration must be between 10 and 300 seconds'}), 400
        
        print(f"→ Starting simulation: {package_path} for {duration}s")
        
        runner = GazeboSimRunner(output_dir=str(app.config['OUTPUT_FOLDER']))
        report = runner.run_simulation(package_path, duration=duration)
        
        print(f"✓ Simulation complete: {report['status']}")
        
        return jsonify({
            'success': True,
            'report': report
        }), 200
        
    except Exception as e:
        print(f"✗ Simulation error: {str(e)}")
        import traceback
        traceback.print_exc()
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/results/<filename>')
def get_result(filename):
    """Serve result files (screenshots, logs)"""
    try:
        return send_from_directory(app.config['OUTPUT_FOLDER'], filename)
    except FileNotFoundError:
        return jsonify({'error': 'File not found'}), 404


@app.route('/api/status')
def status():
    """Health check endpoint"""
    return jsonify({
        'status': 'running',
        'version': '1.0.0',
        'upload_folder': str(app.config['UPLOAD_FOLDER']),
        'output_folder': str(app.config['OUTPUT_FOLDER'])
    })


# Error handlers
@app.errorhandler(413)
def request_entity_too_large(error):
    """Handle file too large error"""
    return jsonify({'error': 'File too large. Maximum size is 50MB'}), 413


@app.errorhandler(404)
def not_found(error):
    """Handle 404 errors"""
    return jsonify({'error': 'Endpoint not found'}), 404


@app.errorhandler(500)
def internal_error(error):
    """Handle internal server errors"""
    return jsonify({'error': 'Internal server error'}), 500


if __name__ == '__main__':
    # Create templates directory if it doesn't exist
    Path('./templates').mkdir(exist_ok=True)
    
    print("="*60)
    print("ROS2 CODE CHECKER - FLASK BACKEND")
    print("="*60)
    print(f"Upload folder: {UPLOAD_FOLDER.absolute()}")
    print(f"Output folder: {OUTPUT_FOLDER.absolute()}")
    print(f"Max upload size: 50MB")
    print("="*60)
    print("\nStarting Flask server...")
    print("Server running at: http://localhost:5000")
    print("Press Ctrl+C to stop\n")
    
    app.run(debug=True, host='0.0.0.0', port=5000)
