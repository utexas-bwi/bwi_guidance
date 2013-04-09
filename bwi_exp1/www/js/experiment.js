
function instructions() {

  var instructions = document.getElementById( 'instructions' );
  var continue_button = document.getElementById( 'continue_button' );

  var host = 'localhost'
  var ros = new ROS('ws://' + host +':9090'); 
  var cmd_vel = null;

  var experiment_lock_service = new ros.Service({
      name        : '/experiment_controller/experiment_lock',
      serviceType : 'bwi_msgs/AcquireExperimentLock'
  });

  var experiment_status_subscriber = new ros.Topic({
    name        : '/experiment_controller/experiment_status',
    messageType : 'bwi_msgs/ExperimentStatus'
  });

  // Any time a message is published to the /chatter topic,
  // the callback will fire.
  experiment_status_subscriber.subscribe(function(message) {
    if (message.locked == true) {
      instructions.innerHTML = "The experiment server is in use!";
      continue_button.disabled = true;
    } else {
      instructions.innerHTML = "The experiment server is free!";
      continue_button.disabled = false;
    }
  });

  continue_button.addEventListener('click', function (event) {
    var request = new ros.ServiceRequest();
    experiment_lock_service.callService(request, function (result) {
      if (result.result) {
        window.location.href="experiment.html?uid=" + result.uid;
      } else {
        alert("Unable to get experiment lock. This can happen if someone tried to enter the experiment the same time as you. Once you close the alert box, the text over the Continue button should change in the next 10 seconds to show the experiment server is in use. If the text still shows the experiment server is in use, please send an email to piyushk@cs.utexas.edu with the text of this alert box. Thanks!!");
      }
    });
  }, false);

}

function start() {

  var score = document.getElementById( 'score' );

  var prmstr = window.location.search.substr(1);
  var prmarr = prmstr.split ("&");
  var params = {};

  for ( var i = 0; i < prmarr.length; i++) {
    var tmparr = prmarr[i].split("=");
    params[tmparr[0]] = tmparr[1];
  }

  // initialize the stream on the canvas
  /* var host = 'zoidberg.csres.utexas.edu' */
  var host = 'localhost'
  var mjpeg = new MjpegCanvas({
    host : host,
      /* topic : '/l_forearm_cam/image_color', */
      topic : '/camera/rgb/image_raw',
      canvasID : 'my-mjpeg',
      /* width : 800, */
      width : 320,
      /* height : 600 */
      height : 240
  });

  var ros = new ROS('ws://' + host +':9090'); 
  var cmd_vel = null;
  ros.on('connection', function() {
    cmd_vel = new ros.Topic({
      name        : '/person/cmd_vel',
      messageType : 'geometry_msgs/Twist'
    });
  });

  var experiment_status_subscriber = new ros.Topic({
    name        : '/experiment_controller/experiment_status',
    messageType : 'bwi_msgs/ExperimentStatus'
  });

  // Any time a message is published to the /chatter topic,
  // the callback will fire.
  experiment_status_subscriber.subscribe(function(message) {
    if (message.locked == true) {
      if (params.uid != message.uid) {
        window.location.href = "index.html";
      }
      score.innerHTML = "Score: " + message.reward;
    } else {
      window.location.href="index.html";
    }
  });

  var prev_velx = 0;
  var prev_vely = 0;
  var prev_vela = 0;
  publishVelocity = function(options) {
    if (options.velx != null)
      prev_velx = options.velx;
    if (options.vely != null)
      prev_vely = options.vely;
    if (options.vela != null)
      prev_vela = options.vela;
    if (cmd_vel != null) {
      var twist = new ros.Message({
        linear: {
          x: prev_velx,
          y: prev_vely,
          z: 0
        },
        angular: {
          x: 0,
          y: 0,
          z: prev_vela
        }
      });
      cmd_vel.publish(twist);
    }
  }

  var arrow  = { turn_left: 37, left: 65, up: 87, turn_right: 39, right: 68, down: 83};
  document.onkeydown = function(event) {
    var keyCode = event.keyCode || event.which;
    if (keyCode === arrow.up) {
      publishVelocity({velx: 1.5});
    }
    else if (keyCode === arrow.down) {
      publishVelocity({velx: -1.5});
    }
    else if (keyCode === arrow.left) {
      publishVelocity({vely: 1.5});
    }
    else if (keyCode === arrow.right) {
      publishVelocity({vely: -1.5});
    }
    else if (keyCode === arrow.turn_left) {
      publishVelocity({vela: 1.5});
    }
    else if (keyCode === arrow.turn_right) {
      publishVelocity({vela: -1.5});
    }
    return false;
  };

  document.onkeyup = function(event) {
    var keyCode = event.keyCode || event.which;
    if (keyCode === arrow.up) {
      publishVelocity({velx: 0});
    }
    else if (keyCode === arrow.down) {
      publishVelocity({velx: 0});
    }
    else if (keyCode === arrow.left) {
      publishVelocity({vely: 0});
    }
    else if (keyCode === arrow.right) {
      publishVelocity({vely: 0});
    }
    else if (keyCode === arrow.turn_left) {
      publishVelocity({vela: 0});
    }
    else if (keyCode === arrow.turn_right) {
      publishVelocity({vela: 0});
    }
    return false;
  };

  var instructions = document.getElementById( 'instructions' );
  var mouse_button = document.getElementById( 'mouse_button' );
  var pause_button = document.getElementById( 'pause_button' );

  // Create a handle for the topic '/chatter' of type std_msgs/String.
  var user_text_subscriber = new ros.Topic({
    name        : '/experiment_controller/user_text',
    messageType : 'std_msgs/String'
  });

  // Any time a message is published to the /chatter topic,
  // the callback will fire.
  user_text_subscriber.subscribe(function(message) {
    // message is an instance of ros.Message.
    instructions.innerHTML = message.data;
  });

  var pause_gazebo = new ros.Service({
      name        : '/gazebo/pause_physics',
      serviceType : 'std_srvs/Empty'
  });
  var unpause_gazebo = new ros.Service({
      name        : '/gazebo/unpause_physics',
      serviceType : 'std_srvs/Empty'
  });

  pause_button.addEventListener('click', function (event) {
    if (pause_button.innerHTML == 'Pause') {
      var request = new ros.ServiceRequest();
      pause_gazebo.callService(request, function (result) {
        pause_button.innerHTML = 'Unpause';
      });
      pause_button.value = 'Unpause';
    } else {
      var request = new ros.ServiceRequest();
      unpause_gazebo.callService(request, function (result) {
        pause_button.innerHTML = 'Pause';
      });
    }
  }, false);

  // http://www.html5rocks.ccuom/en/tutorials/pointerlock/intro/
  // http://mrdoob.github.com/three.js/examples/misc_controls_pointerlock.html

  var havePointerLock = 
    'pointerLockElement' in document || 
    'mozPointerLockElement' in document || 
    'webkitPointerLockElement' in document;

  if ( havePointerLock ) {
    var element = document.body;

    var moveCallback = function ( event ) {
      var movementX = event.movementX || event.mozMovementX || event.webkitMovementX || 0;
      var vela = -movementX / 20.0;
      if (vela > -0.25 && vela < 0.25) {
        vela = 0;
      }
      publishVelocity({vela: vela});
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
    }

    // Hook pointer lock state change events
    document.addEventListener( 'pointerlockchangewindow.location.href=”login.jsp?backurl', pointerlockchange, false );
    document.addEventListener( 'mozpointerlockchange', pointerlockchange, false );
    document.addEventListener( 'webkitpointerlockchange', pointerlockchange, false );

    document.addEventListener( 'pointerlockerror', pointerlockerror, false );
    document.addEventListener( 'mozpointerlockerror', pointerlockerror, false );
    document.addEventListener( 'webkitpointerlockerror', pointerlockerror, false );

    mouse_button.addEventListener( 'click', function ( event ) {

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
    instructions.innerHTML = 'Your browser does not support mouse usage';
  }

};

