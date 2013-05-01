/* http://stackoverflow.com/questions/5448545/how-to-retrieve-get-parameters-from-javascript */
function parseParameters() {
  
  var prmstr = window.location.search.substr(1);
  var prmarr = prmstr.split ("&");
  var params = {};

  for ( var i = 0; i < prmarr.length; i++) {
    var tmparr = prmarr[i].split("=");
    params[tmparr[0]] = tmparr[1];
  }

  return params;
}

/* Global ROS parameters */
var host = 'localhost'
/* var host = 'zoidberg.csres.utexas.edu' */
var ros;
var experiment_status_subscriber; // subs
var update_experiment_service; // services

function initializeROS() {
  /* Setup ROS */
  /* var host = 'localhost' */
  ros = new ROS('ws://' + host +':9090');

  /* Subscribed topics */
  experiment_status_subscriber = new ros.Topic({
    name        : '/experiment_controller/status',
    messageType : 'bwi_msgs/ExperimentStatus'
  });

  /* Service Proxies */
  update_experiment_service = new ros.Service({
      name        : '/experiment_controller/update_experiment',
      serviceType : 'bwi_msgs/UpdateExperiment'
  });

}

/* JS Stuff on the finish page */
function finish() {
  var score_text = document.getElementById( 'score' );
  params = parseParameters();

  var score;
  if (typeof params.score === 'undefined') {
    score = 0;
  } else {
    score = params.score;
  }

  score_text.innerHTML = params.score;
}

/* JS Stuff on the instructions page */
function instructions() {

  /* Get DOM Elements */
  var instructions = document.getElementById('instructions');
  var continue_button = document.getElementById('continue_button');

  initializeROS();

  /* Enable/disable continue button based on the current experiment status.
   * This button is disabled by default, so if the experiment server is not
   * running, nothing will happen */
  experiment_status_subscriber.subscribe(function(message) {
    if (message.locked == true) {
      instructions.innerHTML = "The experiment server is in use!";
      continue_button.disabled = true;
    } else {
      instructions.innerHTML = "The experiment server is free!";
      continue_button.disabled = false;
    }
  });

}

/* Instructions page - Continue to the experiment if you can obtain a lock on 
 * the experiment server */
function attemptExperimentLock() {

  var continue_instructions = 
      document.getElementById('continue_instructions');
  var name_field = document.getElementById('name');

  if (name_field.value == "") {
    continue_instructions.innerHTML = "Please enter your name!"
  } else {
    var request = new ros.ServiceRequest({'lock_experiment': true, 'name': name_field.value});
    update_experiment_service.callService(request, function (result) {
      if (result.result) {
        window.location.href="experiment.html?uid=" + result.uid;
      } else {
        continue_instructions.innerHTML = "It looks like sombody else started"
            + " the experiment at the same time. Please try again later!";
      }
    });
  }
}

function start() {

  /* Get DOM Elements */
  var score = document.getElementById('score');
  var continue_button = document.getElementById('continue_button');
  var instructions = document.getElementById('instructions' );
  var reset_instructions = document.getElementById('reset_instructions');
  var mouse_button = document.getElementById('mouse_button');
  var pause_button = document.getElementById('pause_button');

  params = parseParameters();
  initializeROS();

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
  /* Publishers */

  /* Velocity stuff */
  var cmd_vel_publisher = null;
  ros.on('connection', function() {
    cmd_vel_publisher = new ros.Topic({
      name        : '/person/cmd_vel',
      messageType : 'geometry_msgs/Twist'
    });
  });

  /* Velocity publish convenience function */
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
    if (cmd_vel_publisher != null) {
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
      cmd_vel_publisher.publish(twist);
    }
  }


  /* Keyboard control (velocity + other shortcuts)*/
  var keycode  = { turn_left: 37, left: 65, up: 87, turn_right: 39, right: 68, 
      down: 83, pause: 80, enter:13};
  document.onkeydown = function(event) {
    var keyCode = event.keyCode || event.which;
    if (keyCode === keycode.up) {
      publishVelocity({velx: 1.5});
    }
    else if (keyCode === keycode.down) {
      publishVelocity({velx: -1.5});
    }
    else if (keyCode === keycode.left) {
      publishVelocity({vely: 1.5});
    }
    else if (keyCode === keycode.right) {
      publishVelocity({vely: -1.5});
    }
    else if (keyCode === keycode.turn_left) {
      publishVelocity({vela: 1.5});
    }
    else if (keyCode === keycode.turn_right) {
      publishVelocity({vela: -1.5});
    }
    else if (keyCode === keycode.pause) {
      if (pause_button.disabled == false) { 
        pauseToggle(event);
      }
    }
    else if (keyCode === keycode.enter) {
      if (continue_button.disabled == false) {
        startNextExperiment(event);
      }
    }
    return false;
  };

  document.onkeyup = function(event) {
    var keyCode = event.keyCode || event.which;
    if (keyCode === keycode.up) {
      publishVelocity({velx: 0});
    }
    else if (keyCode === keycode.down) {
      publishVelocity({velx: 0});
    }
    else if (keyCode === keycode.left) {
      publishVelocity({vely: 0});
    }
    else if (keyCode === keycode.right) {
      publishVelocity({vely: 0});
    }
    else if (keyCode === keycode.turn_left) {
      publishVelocity({vela: 0});
    }
    else if (keyCode === keycode.turn_right) {
      publishVelocity({vela: 0});
    }
    return false;
  };

  /* Handle experiment status callback */
  experiment_status_subscriber.subscribe(function(message) {
    if (typeof params.uid === 'undefined' || message.uid == "" || 
        params.uid != message.uid) { // user should not be in experiment page
      window.location.href = "index.html";
    } else {
      if (!message.locked) {
        if (message.previous_experiment_reset) {
          publishVelocity({velx: 0, vely: 0, vela: 0});
          window.location.href = "reset.html";
        } else if (message.previous_experiment_successfully_finished) {
          publishVelocity({velx: 0, vely: 0, vela: 0});
          window.location.href = "end.html?score=" + message.total_reward;
        }
      } else {
        instructions.innerHTML = message.current_display_text;
        if (message.pause_enabled) {
          pause_button.disabled = false;
          if (message.paused) {
            pause_button.innerHTML = "Unpause";
          } else {
            pause_button.innerHTML = "Pause";
          }
        } else {
          pause_button.disabled = true;
        }
        if (message.continue_enabled) {
          continue_button.disabled = false;
        } else {
          continue_button.disabled = true;
        }
        reset_instructions.innerHTML = message.reset_warning_text;
      }
    }
  });

  /* Handle experiment service requests */
  updateExperiment = function(options) {
    var lock_experiment = (options.lock_experiment == null) ? false : options.lock_experiment;
    var name = (options.name == null) ? "" : options.name;
    var email = (options.email == null) ? "" : options.email;
    var pause_experiment = (options.pause_experiment == null) ? false : options.pause_experiment;
    var unpause_experiment = (options.unpause_experiment == null) ? false : options.unpause_experiment;
    var continue_experiment = (options.continue_experiment == null) ? false : options.continue_experiment;
    var request = new ros.ServiceRequest({
        'lock_experiment': lock_experiment, 
        'name': name, 
        'email': email, 
        'pause_experiment': pause_experiment, 
        'unpause_experiment': unpause_experiment,
        'continue_experiment': continue_experiment
    });
    update_experiment_service.callService(request, function (result) {});
  }
  /* Handle continue button */
  var startNextExperiment = function (event) {
    updateExperiment({continue_experiment: true});
  }
  continue_button.addEventListener('click', startNextExperiment, false);

  /* Handle pause button */
  var pauseToggle = function (event) {
    if (pause_button.innerHTML == 'Pause') {
      updateExperiment({pause_experiment: true});
    } else {
      updateExperiment({unpause_experiment: true});
    }
  }
  pause_button.addEventListener('click', pauseToggle, false);


  /* Handle mouse movement based velocity control */

  // http://www.html5rocks.ccuom/en/tutorials/pointerlock/intro/
  // http://mrdoob.github.com/three.js/examples/misc_controls_pointerlock.html
  var havePointerLock = 
    'pointerLockElement' in document || 
    'mozPointerLockElement' in document || 
    'webkitPointerLockElement' in document;

  if (havePointerLock) {
    var element = document.body;

    var moveCallback = function ( event ) {
      var movementX = 
          event.movementX || event.mozMovementX || event.webkitMovementX || 0;
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

    // Hook pointer lock state change events
    document.addEventListener('mozpointerlockchange', pointerlockchange, false);
    document.addEventListener('webkitpointerlockchange', pointerlockchange, 
        false);

    mouse_button.addEventListener( 'click', function ( event ) {

      // Ask the browser to lock the pointer
      element.requestPointerLock = 
      element.requestPointerLock || 
      element.mozRequestPointerLock || 
      element.webkitRequestPointerLock;

      if ( /Firefox/i.test( navigator.userAgent ) ) {
        var fullscreenchange = function ( event ) {
          if (document.fullscreenElement === element || 
              document.mozFullscreenElement === element || 
              document.mozFullScreenElement === element ) {

            document.removeEventListener('fullscreenchange', fullscreenchange);
            document.removeEventListener('mozfullscreenchange', 
                fullscreenchange);
            element.requestPointerLock();
          }
        }

        document.addEventListener('fullscreenchange', fullscreenchange, false);
        document.addEventListener('mozfullscreenchange', fullscreenchange, 
            false);

        element.requestFullscreen = 
            element.requestFullscreen || 
            element.mozRequestFullscreen || 
            element.mozRequestFullScreen || 
            element.webkitRequestFullscreen;
        element.requestFullscreen();
      } else {
        element.requestPointerLock();
      }
    }, false );

  } else {
    mouse_button.disabled = true;
    reset_instructions.innerHTML = "Your browser does not support mouse usage!";
  }
};
