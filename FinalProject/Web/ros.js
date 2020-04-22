let connectionStatus = null;
let ros = null;
let webSocketText = null;
let initError = true;

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
                webSocketText.style.color = "yellow";
                webSocketText.innerHTML = "WebSocket Status: Closed";
            }
            console.log('Connection to websocket server closed.');
        });

    }
    catch (err) {
        console.log(err.message);
    }

    // Check that we connected successfully since apparently ROSLib doesn't throw errors when there's an error connecting...
    if (connectionStatus !== 1) {
        window.alert("Unable to initalize WebSocket connection!");
        webSocketText.style.color = "red";
        webSocketText.innerHTML = "WebSocket Status: Error";
    }
}