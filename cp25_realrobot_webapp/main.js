var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        viewer: null,
        logs: [],
        loading: false,
        isShowCamera: true,
        isShowRobotModel: true,
        service_busy: false,
        rosbridge_address: 'wss://i-085a1df7c42f2495d.robotigniteacademy.com/a4b4c519-01fc-4dae-b102-b3848baf5158/rosbridge/',
        port: '9090',
        // 3D stuff
        viewer3d: null,
        tfClient: null,
        tfClient2: null,
        urdfClient: null,
        tf_aruco: {
            x: 0,
            y: 0,
            z: 0
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

            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
                tfClient.updateGoal()//FIXME undertesting
                tfClient2.updateGoal()
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
                 let without_wss = this.rosbridge_address.split('wss://')[1]
                //let without_ws = this.rosbridge_address.split('ws://')[1]
                console.log(without_wss)
                let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
                //let domain = without_ws.split(':')[0] + ':11315'
                console.log(domain)
                let host = domain + '/cameras'
                this.viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 400,
                height: 300,
                topic: '/D415/color/image_aruco_raw',
                ssl: true,
            })
            }

        },
        disconnect: function() {
            this.ros.close()
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
                rate : 10.0,
                updateDelay : 10,
                republisherUpdateRequested : true, //Fixme try this
                angularThres: 0.001,
                transThres: 0.001,
            })
            this.tfClient2 = new ROSLIB.TFClient({
                ros : this.ros,
                fixedFrame : 'base_link',
                rate : 10.0,
                updateDelay : 10,
                angularThres : 0.001,
                transThres : 0.001
            })
            this.tfClient2.subscribe('aruco_frame', (tf) => {
                console.log(tf.translation.x)
                this.tf_aruco.x = tf.translation.x
                this.tf_aruco.y = tf.translation.y;
                this.tf_aruco.z = tf.translation.z;
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
                scale : 0.1,
                tfClient : this.tfClient,
                rootObject : this.viewer3d.scene,
            });

            var tfAxes2 = new ROS3D.TFAxes({
                frame_id: "D415_color_optical_frame",
                shaftRadius : 0.02,
                headRaidus : 0.07,
                headLength : 0.2,
                scale : 0.2,
                lineType : 'dashed',
                tfClient : this.tfClient,
                rootObject : this.viewer3d.scene,
            });

            var tfAxes3 = new ROS3D.TFAxes({
                frame_id: "aruco_frame",
                shaftRadius : 0.02,
                headRaidus : 0.07,
                headLength : 0.2,
                scale : 0.1,
                tfClient : this.tfClient,
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
    }
})
