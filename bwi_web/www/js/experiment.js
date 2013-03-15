

function start() {

  // initialize the stream on the canvas 
  var mjpeg = new MjpegCanvas({
    host : 'localhost',
      /* topic : '/l_forearm_cam/image_color', */
      topic : '/image_raw',
      canvasID : 'my-mjpeg',
      /* width : 800, */
      width : 320,
      /* height : 600 */
      height : 240
  });

  var ros = new ROS('ws://localhost:9090');
  var cmd_vel = null;
  ros.on('connection', function() {
    cmd_vel = new ros.Topic({
      name        : '/cmd_vel',
      messageType : 'geometry_msgs/Twist'
    });
  });

  publishVelocity = function(options) {
    if (cmd_vel != null) {
      var twist = new ros.Message({
        angular: {
          x: options.velx,
          y: options.vely,
          z: 0
        },
        linear: {
          x: 0,
          y: 0,
          z: options.vela
        }
      });
      cmd_vel.publish(twist);
    }
  }

  // http://www.html5rocks.com/en/tutorials/pointerlock/intro/
  // http://mrdoob.github.com/three.js/examples/misc_controls_pointerlock.html
  
  var instructions = document.getElementById( 'instructions' );

  var havePointerLock = 
    'pointerLockElement' in document || 
    'mozPointerLockElement' in document || 
    'webkitPointerLockElement' in document;

  if ( havePointerLock ) {
    var element = document.body;

    var moveCallback = function ( event ) {
      var movementX = event.movementX || event.mozMovementX || event.webkitMovementX || 0;
      publishVelocity({
        velx: 0,
        vely: 0,
        vela: movementX / 50.0
      });
    }

    var pointerlockchange = function ( event ) {
      if ( document.pointerLockElement === element || 
           document.mozPointerLockElement === element || 
           document.webkitPointerLockElement === element ) {

        document.addEventListener("mousemove", moveCallback, false);
      } else {
        document.removeEventListener("mousemove", moveCallback, false);
      }
    }

    var pointerlockerror = function ( event ) {
      instructions.style.display = '';
    }

    // Hook pointer lock state change events
    document.addEventListener( 'pointerlockchange', pointerlockchange, false );
    document.addEventListener( 'mozpointerlockchange', pointerlockchange, false );
    document.addEventListener( 'webkitpointerlockchange', pointerlockchange, false );

    document.addEventListener( 'pointerlockerror', pointerlockerror, false );
    document.addEventListener( 'mozpointerlockerror', pointerlockerror, false );
    document.addEventListener( 'webkitpointerlockerror', pointerlockerror, false );

    instructions.addEventListener( 'click', function ( event ) {

      // Ask the browser to lock the pointer
      element.requestPointerLock = 
      element.requestPointerLock || 
      element.mozRequestPointerLock || 
      element.webkitRequestPointerLock;

    if ( /Firefox/i.test( navigator.userAgent ) ) {

      var fullscreenchange = function ( event ) {

        if ( document.fullscreenElement === element || document.mozFullscreenElement === element || document.mozFullScreenElement === element ) {

          document.removeEventListener( 'fullscreenchange', fullscreenchange );
          document.removeEventListener( 'mozfullscreenchange', fullscreenchange );

          element.requestPointerLock();
        }

      }

      document.addEventListener( 'fullscreenchange', fullscreenchange, false );
      document.addEventListener( 'mozfullscreenchange', fullscreenchange, false );

      element.requestFullscreen = element.requestFullscreen || element.mozRequestFullscreen || element.mozRequestFullScreen || element.webkitRequestFullscreen;

      element.requestFullscreen();

    } else {

      element.requestPointerLock();

    }

    }, false );

  } else {
    instructions.innerHTML = 'Your browser cannot run this experiment.';
  }

};

