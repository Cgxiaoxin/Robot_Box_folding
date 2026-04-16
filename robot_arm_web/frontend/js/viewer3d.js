import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';

console.log('[Viewer3D] Script loaded and executing.');

/**
 * 3D Viewer for Dual Robot Arm Digital Twin
 */
class Viewer3D {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        if (!this.container) {
            console.error(`[Viewer3D] Container #${containerId} not found.`);
            return;
        }

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0f172a); // Match slate-900 theme
        
        // Add subtle fog to blend into background
        this.scene.fog = new THREE.FogExp2(0x0f172a, 0.05);

        // Camera
        this.camera = new THREE.PerspectiveCamera(
            45,
            this.container.clientWidth / this.container.clientHeight,
            0.1,
            100
        );
        // Position camera to see both arms
        this.camera.position.set(2, 1.5, 3);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ 
            canvas: document.getElementById('viewer3d-canvas'),
            antialias: true,
            alpha: true
        });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

        // Controls
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.target.set(0, 0.5, 0);

        // Lighting
        this.setupLighting();

        // Floor / Grid
        const gridHelper = new THREE.GridHelper(5, 50, 0x334155, 0x1e293b);
        gridHelper.position.y = -0.01;
        this.scene.add(gridHelper);

        // Axes (X=red, Y=green, Z=blue)
        const axesHelper = new THREE.AxesHelper(0.5);
        this.scene.add(axesHelper);

        // URDF Model
        this.robot = null;
        
        // Joint mappings mapping IDs to node objects
        this.jointNodes = {
            left: {},
            right: {}
        };

        this.initLoader();

        // Resize handler
        window.addEventListener('resize', this.onWindowResize.bind(this));

        // Start render loop
        this.animate = this.animate.bind(this);
        this.animate();
        
        // Hide overlay initially, will show status if loading fails
        const overlay = document.getElementById('viewer3d-overlay');
        if (overlay) overlay.textContent = '3D 孪生引擎 (初始化中...)';
    }

    setupLighting() {
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);

        const dirLight = new THREE.DirectionalLight(0xffffff, 1.0);
        dirLight.position.set(5, 10, 5);
        dirLight.castShadow = true;
        dirLight.shadow.mapSize.width = 2048;
        dirLight.shadow.mapSize.height = 2048;
        this.scene.add(dirLight);

        const fillLight = new THREE.DirectionalLight(0x90b0d0, 0.5);
        fillLight.position.set(-5, 3, -5);
        this.scene.add(fillLight);
    }

    initLoader() {
        const _this = this;
        const manager = new THREE.LoadingManager();
        const loader = new URDFLoader(manager);
        
        // Custom parsing to handle STL/DAE/OBJ inside URDF correctly using basic Three.js loaders if needed.
        // The urdf-loader automatically loads STL if STLLoader is provided or built-in in its full version.
        // It relies on loaders registered on the URDFLoader instance or global THREE namespace.
        
        const overlay = document.getElementById('viewer3d-overlay');
        
        // Map package://lens_dual_arm_description to /urdf/
        loader.packages = {
            'lens_dual_arm_description': '/urdf/'
        };
        
        loader.load(
            '/urdf/urdf/lens_dual_arm_description.urdf',
            (robot) => {
                _this.robot = robot;
                
                // Adjust model orientation if necessary
                // Usually URDF Z is up, Three.js Y is up. The loader handles this by default.
                
                // Enable shadows for the robot
                robot.traverse(c => {
                    if (c.isMesh) {
                        c.castShadow = true;
                        c.receiveShadow = true;
                        
                        // Improve material properties for the dark slate theme
                        if(c.material) {
                            c.material.roughness = 0.4;
                            c.material.metalness = 0.2;
                        }
                    }
                });

                _this.scene.add(robot);
                
                // Cache joint nodes for fast lookup
                _this.cacheJoints(robot);

                if (overlay) overlay.style.display = 'none';
                console.log('[Viewer3D] URDF Model loaded successfully.', robot);
            },
            (progress) => {
                if (overlay && progress && progress.lengthComputable && progress.total > 0) {
                    const percent = (progress.loaded / progress.total * 100).toFixed(0);
                    overlay.textContent = `加载 3D 模型... ${percent}%`;
                } else if (overlay) {
                    overlay.textContent = `加载 3D 模型...`;
                }
            },
            (error) => {
                console.error('[Viewer3D] Failed to load URDF model:', error);
                if (overlay) {
                    overlay.textContent = '3D 模型加载失败';
                    overlay.style.color = 'var(--danger)';
                }
            }
        );
    }

    cacheJoints(robot) {
        // Map of standard URDF joint names
        const LEFT_JOINT_NAMES = [
            'Left_Shoulder_Pitch_Joint', // 51
            'Left_Shoulder_Roll_Joint',  // 52
            'Left_Shoulder_Yaw_Joint',   // 53
            'Left_Elbow_Pitch_Joint',    // 54
            'Left_Wrist_Yaw_Joint',      // 55
            'Left_Wrist_Pitch_Joint',    // 56
            'Left_Wrist_Roll_Joint',     // 57
        ];
        
        const RIGHT_JOINT_NAMES = [
            'Right_Shoulder_Pitch_Joint', // 61
            'Right_Shoulder_Roll_Joint',  // 62
            'Right_Shoulder_Yaw_Joint',   // 63
            'Right_Elbow_Pitch_Joint',    // 64
            'Right_Wrist_Yaw_Joint',      // 65
            'Right_Wrist_Pitch_Joint',    // 66
            'Right_Wrist_Roll_Joint',     // 67
        ];

        // Map IDs to references
        LEFT_JOINT_NAMES.forEach((name, index) => {
            const id = 51 + index;
            const jointNode = robot.joints[name];
            if (jointNode) {
                this.jointNodes.left[id] = jointNode;
            } else {
                console.warn(`[Viewer3D] Joint ${name} not found in URDF!`);
            }
        });

        RIGHT_JOINT_NAMES.forEach((name, index) => {
            const id = 61 + index;
            let jointNode = robot.joints[name];
            if (jointNode) {
                this.jointNodes.right[id] = jointNode;
            } else {
                console.warn(`[Viewer3D] Joint ${name} not found in URDF!`);
            }
        });
    }

    /**
     * Update joints from the WebSocket state
     * @param {Object} state - The global appState
     */
    updateState(state) {
        if (!this.robot) return;

        // Apply Left Arm Positions
        const leftMotors = state.arms?.left?.motors;
        if (leftMotors) {
            for (let id = 51; id <= 57; id++) {
                const pos = leftMotors[id]?.position;
                if (pos !== undefined && this.jointNodes.left[id]) {
                    // Update joint angle
                    this.jointNodes.left[id].setJointValue(pos);
                }
            }
        }

        // Apply Right Arm Positions
        const rightMotors = state.arms?.right?.motors;
        if (rightMotors) {
            for (let id = 61; id <= 67; id++) {
                const pos = rightMotors[id]?.position;
                if (pos !== undefined && this.jointNodes.right[id]) {
                    // Note: If physical right arm is mirrored or inverted (like 61 or 66), 
                    // we might need to invert it here if the URDF uses different conventions.
                    // Assuming URDF matches kinematic calculations in backend for now.
                    this.jointNodes.right[id].setJointValue(pos);
                }
            }
        }
    }

    onWindowResize() {
        if (!this.container) return;
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }

    animate() {
        requestAnimationFrame(this.animate);
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

// Instantiate and expose to global window so main.js can access it
// type="module" implies defer, so the DOM is already ready when this runs.
window.Viewer3D = new Viewer3D('viewer3d-container');
