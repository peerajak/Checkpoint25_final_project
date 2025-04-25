var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        checkbox_fixTf: false,
        checkbox_fixTf_string: "Calibrating",
        checkbox_hole_fixTf: false,
        checkbox_hole_fixTf_string: "Detecting Holes",
        ros: null,
        viewer: null,
        logs: [],
        loading: false,
        isShowCamera: true,
        isShowRobotModel: true,
        service_busy: false,
        rosbridge_address: 'ws://localhost:9090',
        port: '9090',
        // 3D stuff
        viewer3d: null,
        tfClient: null,
        tfClient_aruco_camera: null,
        tfClient_aruco_baselink: null,
        tfClient_hole_baselink: null,
        tfClient_camera_sol_baselink: null,
        tfClient_camera_real_baselink: null,
        tfClient_detected_vs_real_aruco: null,
        urdfClient: null,
        calibrate_object_listener: null,
        camera_info_listener: null,
        calibrate_object_info: {
            u1: 0,
            v1: 0,
            p1x: 0,
            p1y: 0,
            p1z: 0,
            u2: 0,
            v2: 0,
            p2x: 0,
            p2y: 0,
            p2z: 0,
            u3: 0,
            v3: 0,
            p3x: 0,
            p3y: 0,
            p3z: 0,
            u4: 0,
            v4: 0,
            p4x: 0,
            p4y: 0,
            p4z: 0,
        },
        camera_info: {
            cx: 0,
            cy: 0,
            fx: 0,
            fy: 0,

        },
        tf_aruco_camera: {
            x: 0,
            y: 0,
            z: 0,
            ax: 0,
            ay: 0,
            az: 0,
            aw: 0
        },
        tf_aruco_baselink: {
            x: 0,
            y: 0,
            z: 0,
            ax: 0,
            ay: 0,
            az: 0,
            aw: 0
        },
        tf_hole_baselink: {
            x: 0,
            y: 0,
            z: 0,
            ax: 0,
            ay: 0,
            az: 0,
            aw: 0
        },
        tf_camera_sol_baselink: {
            x: 0,
            y: 0,
            z: 0,
            ax: 0,
            ay: 0,
            az: 0,
            aw: 0
        },
        tf_camera_real_baselink: {
            x: 0,
            y: 0,
            z: 0,
            ax: 0,
            ay: 0,
            az: 0,
            aw: 0
        },
        tf_error_cal: {
            x: 0,
            y: 0,
            z: 0,
            ax: 0,
            ay: 0,
            az: 0,
            aw: 0
        },
    },
    // helper methods to connect to ROS
    methods: {
        connect: function() {
            this.logs.unshift('connecting...')
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })
            
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false                
                this.showCamera()
                this.showRobotModel()
                this.callPlanningSceneService()
                this.camera_info_listener = new ROSLIB.Topic({
                    ros : this.ros,
                    name : '/wrist_rgbd_depth_sensor/camera_info',
                    messageType : 'sensor_msgs/CameraInfo'
                });

                this.camera_info_listener.subscribe((message) => {
                    this.camera_info.cx = message.K[2]
                    this.camera_info.cy = message.K[5]
                    this.camera_info.fx = message.K[0]
                    this.camera_info.fy = message.K[4]

                });

                this.calibrate_object_listener = new ROSLIB.Topic({
                    ros : this.ros,
                    name : '/wrist_rgbd_depth_sensor/calibrate_object',
                    messageType : 'std_msgs/Float32MultiArray'
                });

                this.calibrate_object_listener.subscribe((message) => {
                    //console.log(message)
                    //console.log.unshift('Received calibrate_object message on ' + listener.name + ': ' + message.data[0] + "," + message.data[1] + "," );
                    this.calibrate_object_info.u1 = message.data[0]
                    this.calibrate_object_info.v1 = message.data[1]
                    this.calibrate_object_info.p1x = message.data[2]
                    this.calibrate_object_info.p1y = message.data[3]
                    this.calibrate_object_info.p1z = message.data[4]

                    this.calibrate_object_info.u2 = message.data[5]
                    this.calibrate_object_info.v2 = message.data[6]
                    this.calibrate_object_info.p2x = message.data[7]
                    this.calibrate_object_info.p2y = message.data[8]
                    this.calibrate_object_info.p2z = message.data[9]

                    this.calibrate_object_info.u3 = message.data[10]
                    this.calibrate_object_info.v3 = message.data[11]
                    this.calibrate_object_info.p3x = message.data[12]
                    this.calibrate_object_info.p3y = message.data[13]
                    this.calibrate_object_info.p3z = message.data[14]

                    this.calibrate_object_info.u4 = message.data[15]
                    this.calibrate_object_info.v4 = message.data[16]
                    this.calibrate_object_info.p4x = message.data[17]
                    this.calibrate_object_info.p4y = message.data[18]
                    this.calibrate_object_info.p4z = message.data[19]

                });

            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false   
                this.unset3DViewer()  
                this.unsetCamera()

                clearInterval(this.pubInterval)         
            })
        },



        showCamera: function() {
            this.setCamera()
        },
        closeCamera: function() {
            this.unsetCamera()
        },
        showRobotModel: function() {
            this.setup3DViewer()
        },
        closeRobotModel: function () {
            this.unset3DViewer()  
        },

        setCamera: function() {

            if(this.viewer == null){
                // let without_wss = this.rosbridge_address.split('ws://')[1]
                let without_ws = this.rosbridge_address.split('ws://')[1]
                // console.log(without_wss)
                // let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
                let domain = without_ws.split(':')[0] + ':11315'
                console.log(domain)
                let host = domain //+ '/cameras'
                this.viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 400,
                height: 300,
                topic: '/wrist_rgbd_depth_sensor/image_hole_frame',
                ssl: false,
            })
            }

        },
        disconnect: function() {
            this.ros.close()
        },
    
        insertItemIntoListBox: function(){
            var x = document.getElementById("access");
            //TODO item should contains current 6 joint_states
            var item = [this.tf_aruco_baselink.x.toFixed(3),this.tf_aruco_baselink.y.toFixed(3) ,this.tf_aruco_baselink.z.toFixed(3),
            this.tf_aruco_baselink.ax.toFixed(3),this.tf_aruco_baselink.ay.toFixed(3) ,this.tf_aruco_baselink.az.toFixed(3), this.tf_aruco_baselink.aw.toFixed(3)];
            var option = document.createElement("option");
            option.text = item;
            
            if(x.options.length >= 4){
               x.remove(0);
            }
            x.add(option);
        },

        removeSelectedItemIntoListBox: function(){
            var x = document.getElementById("access");
            x.remove(x.selectedIndex);
        },
        clearItemIntoListBox: function(){
            var x = document.getElementById("access");
            while (x.options.length > 0) {                
                x.remove(0);
            } 
        },
        addPoseManually: function(){
            var x = document.getElementById("access");
            var y = document.getElementById("manual_pose")
            var option = document.createElement("option");
            option.text = y.value;
            x.add(option);
        },

        sendPoseCommand: function(pose_aruco) {
            console.log("sendPoseCommand" , parseFloat(pose_aruco[0]));
            pose_aruco_0 =  parseFloat(pose_aruco[0]);
            pose_aruco_1 =  parseFloat(pose_aruco[1]);
            pose_aruco_2 =  parseFloat(pose_aruco[2]);
            pose_aruco_3 =  parseFloat(pose_aruco[3]);
            pose_aruco_4 =  parseFloat(pose_aruco[4]);
            pose_aruco_5 =  parseFloat(pose_aruco[5]);
            pose_aruco_6 =  parseFloat(pose_aruco[6]);
            let topic2 = new ROSLIB.Topic({
                ros: this.ros,
                name: '/moveit_goto_pose',
                messageType: 'geometry_msgs/Pose'
            })
            let message = new ROSLIB.Message({
                position: {x: pose_aruco_0,y: pose_aruco_1,z: pose_aruco_2},
                orientation: {x: pose_aruco_3,y: pose_aruco_4,z: pose_aruco_5,w: pose_aruco_6}
            })
            topic2.publish(message)
        },
        moveEndToWaypoints: function(){
            var x = document.getElementById("access");
            if(x.options.length > 0){
                const array_pose_aruco = x.options[x.selectedIndex].text.split(',');
                console.log("joints are: " + array_pose_aruco);     
                this.sendPoseCommand(array_pose_aruco);   
            }

        },
        fixCalibrationCheckbox() {
       
        if(!this.checkbox_fixTf) {
          this.checkbox_fixTf_string = "Camera TF fixed"
          this.fix_tf()
        }else{
          this.checkbox_fixTf_string = "Calibrating"
          this.calibrate_tf()
        }
        console.log("fixCalibrationCheckbox",this.checkbox_fixTf)
        this.$emit('setCheckboxVal', this.checkbox_fixTf)
        },
        fix_tf: function() {
            console.log("fix_tf")
            this.service_busy = true
            this.service_response = ''
            // define the service to be called
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: '/tf2_pub_service',
                serviceType: 'std_srvs/SetBool',
            })

            // define the request
            let request = new ROSLIB.ServiceRequest({
                data: false,
            })

            // define a callback
            service.callService(request, (result) => {
                this.service_busy = false
                this.service_response = JSON.stringify(result)
                console.log(result)
            }, (error) => {
                this.service_busy = false
                console.error(error)
            })
        },
        calibrate_tf: function() {
            console.log("calibrate_tf")
            this.service_busy = true
            this.service_response = ''
            // define the service to be called
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: '/tf2_pub_service',
                serviceType: 'std_srvs/SetBool',
            })

            // define the request
            let request = new ROSLIB.ServiceRequest({
                data: true,
            })

            // define a callback
            service.callService(request, (result) => {
                this.service_busy = false
                this.service_response = JSON.stringify(result)
                console.log(result)
            }, (error) => {
                this.service_busy = false
                console.error(error)
            })
        },
        fixHoleCheckbox() {

        if(!this.checkbox_hole_fixTf) {
          this.checkbox_hole_fixTf_string = "A hole is TF fixed"
          this.fix_hole_tf()
        }else{
          this.checkbox_hole_fixTf_string = "Detecting Holes"
          this.calibrate_hole_tf()
        }
        console.log("fixCalibrationCheckbox",this.checkbox_hole_fixTf)
        this.$emit('setCheckboxVal', this.checkbox_hole_fixTf)
        },
        fix_hole_tf: function() {
            console.log("fix_hole_tf")
            this.service_busy = true
            this.service_response = ''
            // define the service to be called
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: '/change_hole_mode_service',
                serviceType: 'std_srvs/SetBool',
            })

            // define the request
            let request = new ROSLIB.ServiceRequest({
                data: false,
            })

            // define a callback
            service.callService(request, (result) => {
                this.service_busy = false
                this.service_response = JSON.stringify(result)
                console.log(result)
            }, (error) => {
                this.service_busy = false
                console.error(error)
            })
        },
        calibrate_hole_tf: function() {
            console.log("calibrate_hole_tf")
            this.service_busy = true
            this.service_response = ''
            // define the service to be called
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: '/change_hole_mode_service',
                serviceType: 'std_srvs/SetBool',
            })

            // define the request
            let request = new ROSLIB.ServiceRequest({
                data: true,
            })

            // define a callback
            service.callService(request, (result) => {
                this.service_busy = false
                this.service_response = JSON.stringify(result)
                console.log(result)
            }, (error) => {
                this.service_busy = false
                console.error(error)
            })
        },
        setup3DViewer() {
            this.viewer3d = new ROS3D.Viewer({
                background: '#cccccc',
                divID: 'div3DViewer',
                width: 400,
                height: 400,
                antialias: true,
                fixedFrame: 'odom',
                axesDisplay : true,
                cameraPose: {
                    x: -2,
                    y: 0,
                    z: 1.0
                  }
            })

            // Add a grid.
            this.viewer3d.addObject(new ROS3D.Grid({
                color:'#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))
            
            // Setup a client to listen to TFs.
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0
            })
            this.tfClient_aruco_camera = new ROSLIB.TFClient({
                ros : this.ros,
                fixedFrame : 'wrist_rgbd_camera_depth_optical_frame',
                angularThres : 0.01,
                transThres : 0.01,
                seconds: 1.0
                
            })
            this.tfClient_aruco_camera.subscribe('aruco_frame', (tf) => {
                this.tf_aruco_camera.x = tf.translation.x
                this.tf_aruco_camera.y = tf.translation.y;
                this.tf_aruco_camera.z = tf.translation.z;
                this.tf_aruco_camera.ax = tf.rotation.x
                this.tf_aruco_camera.ay = tf.rotation.y;
                this.tf_aruco_camera.az = tf.rotation.z;
                this.tf_aruco_camera.aw = tf.rotation.w;
            })
            this.tfClient_aruco_baselink = new ROSLIB.TFClient({
                ros : this.ros,
                fixedFrame : 'base_link',
                angularThres : 0.01,
                transThres : 0.01,
                seconds: 1.0
                
            })
            this.tfClient_aruco_baselink.subscribe('aruco_frame', (tf) => {
                this.tf_aruco_baselink.x = tf.translation.x
                this.tf_aruco_baselink.y = tf.translation.y;
                this.tf_aruco_baselink.z = tf.translation.z;
                this.tf_aruco_baselink.ax = tf.rotation.x
                this.tf_aruco_baselink.ay = tf.rotation.y;
                this.tf_aruco_baselink.az = tf.rotation.z;
                this.tf_aruco_baselink.aw = tf.rotation.w;
            })
            this.tfClient_hole_baselink = new ROSLIB.TFClient({
                ros : this.ros,
                fixedFrame : 'base_link',
                angularThres : 0.01,
                transThres : 0.01,
                seconds: 1.0
                
            })
            this.tfClient_hole_baselink.subscribe('hole_frame', (tf) => {
                this.tf_hole_baselink.x = tf.translation.x
                this.tf_hole_baselink.y = tf.translation.y;
                this.tf_hole_baselink.z = tf.translation.z;
                this.tf_hole_baselink.ax = tf.rotation.x
                this.tf_hole_baselink.ay = tf.rotation.y;
                this.tf_hole_baselink.az = tf.rotation.z;
                this.tf_hole_baselink.aw = tf.rotation.w;
            })
            this.tfClient_camera_sol_baselink = new ROSLIB.TFClient({
                ros : this.ros,
                fixedFrame : 'base_link',
                rate : 10.0,
                updateDelay : 10,
                angularThres : 0.01,
                transThres : 0.01,
                seconds: 1.0
            })
            this.tfClient_camera_sol_baselink.subscribe('camera_solution_frame', (tf) => {
                this.tf_camera_sol_baselink.x = tf.translation.x
                this.tf_camera_sol_baselink.y = tf.translation.y;
                this.tf_camera_sol_baselink.z = tf.translation.z;
                this.tf_camera_sol_baselink.ax = tf.rotation.x
                this.tf_camera_sol_baselink.ay = tf.rotation.y;
                this.tf_camera_sol_baselink.az = tf.rotation.z;
                this.tf_camera_sol_baselink.aw = tf.rotation.w;
            })
            this.tfClient_camera_real_baselink = new ROSLIB.TFClient({
                ros : this.ros,
                fixedFrame : 'base_link',
                rate : 10.0,
                updateDelay : 10,
                angularThres : 0.01,
                transThres : 0.01,
                seconds: 1.0
            })
            this.tfClient_camera_real_baselink.subscribe('wrist_rgbd_camera_depth_optical_frame', (tf) => {
                this.tf_camera_real_baselink.x = tf.translation.x
                this.tf_camera_real_baselink.y = tf.translation.y;
                this.tf_camera_real_baselink.z = tf.translation.z;
                this.tf_camera_real_baselink.ax = tf.rotation.x
                this.tf_camera_real_baselink.ay = tf.rotation.y;
                this.tf_camera_real_baselink.az = tf.rotation.z;
                this.tf_camera_real_baselink.aw = tf.rotation.w;
            })
            this.tfClient_detected_vs_real_aruco = new ROSLIB.TFClient({
                ros : this.ros,
                fixedFrame : 'base_link',
                rate : 10.0,
                updateDelay : 10,
                angularThres : 0.001,
                transThres : 0.001
            })
            this.tfClient_detected_vs_real_aruco.subscribe('rg2_gripper_aruco_link', (tf) => {
                console.log('tfClient_detected_vs_real_aruco sub aruco_frame')
                console.log(tf.translation.x,tf.translation.y,tf.translation.z)
                this.tf_error_cal.x = tf.translation.x
                this.tf_error_cal.y = tf.translation.y;
                this.tf_error_cal.z = tf.translation.z;
                this.tf_error_cal.ax = tf.rotation.x
                this.tf_error_cal.ay = tf.rotation.y;
                this.tf_error_cal.az = tf.rotation.z;
                this.tf_error_cal.aw = tf.rotation.w;
            })

            // Setup the URDF client.
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: 'robot_description',
                tfClient: this.tfClient,
                // We use "path: location.origin + location.pathname"
                // instead of "path: window.location.href" to remove query params,
                // otherwise the assets fail to load
                path: location.origin + location.pathname,
                rootObject: this.viewer3d.scene,
                loader: ROS3D.COLLADA_LOADER_2
            })
            
            //Setup TF Axes visualizer
            var tfAxes1 = new ROS3D.TFAxes({
                frame_id: "base_link",
                shaftRadius : 0.02,
                headRaidus : 0.07,
                headLength : 0.2,
                scale : 0.3,
                tfClient : this.tfClient,
                rootObject : this.viewer3d.scene,
            });

            var tfAxes2 = new ROS3D.TFAxes({
                frame_id: "camera_solution_frame",
                shaftRadius : 0.02,
                headRaidus : 0.07,
                headLength : 0.2,
                scale : 0.1,
                tfClient : this.tfClient_camera_sol_baselink,
                rootObject : this.viewer3d.scene,
            });

            var tfAxes3 = new ROS3D.TFAxes({
                frame_id: "aruco_frame",
                shaftRadius : 0.02,
                headRaidus : 0.07,
                headLength : 0.2,
                scale : 0.1,
                tfClient : this.tfClient_aruco_baselink,
                rootObject : this.viewer3d.scene,
            });

            var tfAxes4 = new ROS3D.TFAxes({
                frame_id: "rg2_gripper_aruco_link",
                shaftRadius : 0.02,
                headRaidus : 0.07,
                headLength : 0.2,
                scale : 0.1,
                tfClient : this.tfClient_detected_vs_real_aruco,
                rootObject : this.viewer3d.scene,
            });

            var tfAxes5 = new ROS3D.TFAxes({
                frame_id: "wrist_rgbd_camera_depth_optical_frame",
                shaftRadius : 0.02,
                headRaidus : 0.07,
                headLength : 0.2,
                scale : 0.1,
                tfClient : this.tfClient_camera_real_baselink,
                rootObject : this.viewer3d.scene,
            });

            var tfAxes6 = new ROS3D.TFAxes({
                frame_id: "hole_frame",
                shaftRadius : 0.02,
                headRaidus : 0.07,
                headLength : 0.2,
                scale : 0.1,
                tfClient : this.tfClient_hole_baselink,
                rootObject : this.viewer3d.scene,
            });

        },


        unsetCamera() {
            document.getElementById('divCamera').innerHTML = ''
            this.viewer = null
        },
        unset3DViewer() {
            document.getElementById('div3DViewer').innerHTML = ''
            this.viewer3d = null
        },
        callPlanningSceneService: function() {
            // service is busy
            this.service_busy = true
            this.service_response = ''
            // define the service to be called
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: '/planning_scene_cp25',
                serviceType: 'std_srvs/Empty',
            })
            // define the request
            let request = new ROSLIB.ServiceRequest({
                
            })

            // define a callback
            service.callService(request, (result) => {
                this.service_busy = false
                this.service_response = JSON.stringify(result)
            }, (error) => {
                this.service_busy = false
                console.error(error)
            })
        },
        callMoveitSimService: function(toPos) {
            // service is busy
            this.service_busy = true
            this.service_response = ''
            // define the service to be called
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: '/moveit_sim_service',
                serviceType: 'std_srvs/SetBool',
            })

            // define the request
            let request = new ROSLIB.ServiceRequest({
                data: toPos == "goShow",
            })

            // define a callback
            service.callService(request, (result) => {
                this.service_busy = false
                this.service_response = JSON.stringify(result)
            }, (error) => {
                this.service_busy = false
                console.error(error)
            })
        },
        callMoveitSimHoleService: function() {
            // service is busy
            this.service_busy = true
            this.service_response = ''
            // define the service to be called
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: '/moveit_sim_hole_service',
                serviceType: 'std_srvs/SetBool',
            })

            // define the request
            let request = new ROSLIB.ServiceRequest({
                data: true,
            })

            // define a callback
            service.callService(request, (result) => {
                this.service_busy = false
                this.service_response = JSON.stringify(result)
            }, (error) => {
                this.service_busy = false
                console.error(error)
            })
        },
    }
})