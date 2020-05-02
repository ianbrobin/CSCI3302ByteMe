let connectionStatus = 1;
let ros = null;
let webSocketText = null;
let initError = true;
let gameCompleteSub = null;
let robotTurnSub = null;
let submitTurnPub = null;
let gameCompletePub = null;
let gameResetPub = null;

// Set-up all our ROS stuff
function initializeROS() {
    try {
        webSocketText = document.getElementById('webSocketStatus');

        ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        // Set-up Successful connection callback
        ros.on('connection', function() {
            connectionStatus = 1;
            initError = false;
            webSocketText.style.color = "green";
            webSocketText.innerHTML = "WebSocket Status: Connected";
            console.log('Connected to websocket server.');
        });

        // Set-up Error Connection callback
        ros.on('error', function(error) {
            connectionStatus = -1;
            webSocketText.style.color = "red";
            webSocketText.innerHTML = "WebSocket Status: Error";
            console.log('Error connecting to websocket server: ', error);
        });

        // Set-up Closed Connection Callback
        ros.on('close', function() {
            connectionStatus = 0;

            // This is called instead of the error callback when the WebSocket server isn't even running
            // so this if statement makes it so the Error Text is shown instead of the Closed text
            // if we encounter an issue on launch...
            if (!initError) {
                webSocketText.style.color = "#CFB87C";
                webSocketText.innerHTML = "WebSocket Status: Closed";
            }
            console.log('Connection to websocket server closed.');
        });

    }
    catch (err) {
        console.log(err.message);
    }

    // Check that we connected successfully since apparently ROSLib doesn't throw errors when there's an error connecting...
    if (connectionStatus != 1) {
        window.alert("Unable to initalize WebSocket connection!");
        webSocketText.style.color = "red";
        webSocketText.innerHTML = "WebSocket Status: Error";
    }

	// Set-up all of our subscribers
	// GameCompleteSub
	gameCompleteSub = new ROSLIB.Topic({
		ros: ros,
		name: '/TicTac/GameCompleted',
		messageType: 'std_msgs/String'
	});	
	// GameCompleteSub set-up Callback
	gameCompleteSub.subscribe(function(msg) {
		console.log("Received " + msg.data);
		checkWinner();
	});
	
	// RobotTurnSub
	robotTurnSub = new ROSLIB.Topic({
		ros: ros,
		name: '/TicTac/RobotTurnSubmitted',
		messageType: 'std_msgs/String'
	});
	robotTurnSub.subscribe(function(msg) {
		console.log("Robot submitted " + msg.data);
		makeMove(msg.data, 'sim');
	});


	// Set-up our publishers
	submitTurnPub = new ROSLIB.Topic({
		ros: ros,
		name: '/TicTac/HumanTurnSubmitted',
		messageType: 'std_msgs/String'
	});
	
	gameCompletePub = new ROSLIB.Topic({
		ros: ros,
		name: '/TicTac/GameCompleted',
		messageType: 'std_msgs/String'
	});

	gameResetPub = new ROSLIB.Topic({
		ros: ros,
		name: '/TicTac/reset',
		messageType: 'geometry_msgs/Pose2D'
	});

}


function publishMove(cell) {
	let msg = new ROSLIB.Message({
		data: cell
	});

	submitTurnPub.publish(msg);

	console.log('Sent Message with Human Move = ' + cell);
}

function publishGameComplete(winner) {
	let msg = new ROSLIB.Message({
		data: winner
	});
	gameCompletePub.publish(msg);
	console.log('Sent gameComplete message with winner = ' + winner);
}

function publishGameReset() {
	let msg = new ROSLIB.Message({
		x: 0.0,
		y: 0.0,
		theta: 0.0
	});
	gameResetPub.publish(msg);
	console.log('Sent gameReset');
}

