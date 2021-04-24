// Javascript code to set up a websocket.

var m_url_JS = "ws://nanostat.local:81/";
//var m_url_JS = "ws://192.168.1.44:81/";
// var url = "ws://192.168.4.1:1337/";

var m_canvas_JS;
var context;

// This is called when the page finishes loading
function init() {

    // Assign page elements to variables
    m_canvas_JS = document.getElementById("m_canvas");

    // Connect to WebSocket server
    wsConnect(m_url_JS);
}

// Call this to connect to the WebSocket server
function wsConnect(m_url_JS) {

    // Connect to WebSocket server
    m_url_JS="ws://" + window.location.hostname + ":81/"
    // console.log(m_url_JS);
    m_websocket = new WebSocket(m_url_JS);

    // Assign callbacks
    m_websocket.onopen = function (evt) { onOpen(evt) };
    m_websocket.onclose = function (evt) { onClose(evt) };
    m_websocket.onmessage = function (evt) { onMessage(evt) };
    m_websocket.onerror = function (evt) { onError(evt) };

}

// Called when a WebSocket connection is established with the server
function onOpen(evt) {

    // Log connection state
    console.log("Connected");

    // Enable button
    // button.disabled = false;

    // Get the current state of the LED
    // doSend("getLEDState");
}

// Called when the WebSocket connection is closed
function onClose(evt) {

    // Log disconnection state
    console.log("Disconnected");

    // Disable button
    // button.disabled = true;

    // Try to reconnect after a few seconds
    setTimeout(function () { wsConnect(m_url_JS) }, 2000);
}

// Called when a message is received from the server
function onMessage(evt) {

    // Print out our received message
    console.log("Received: " + evt.data);

    // Update circle graphic with LED state
    // switch (evt.data) {
    //   case "0":
    //     console.log("LED is off");
    //     context.fillStyle = "black";
    //     context.fill();
    //     break;
    //   case "1":
    //     console.log("LED is on");
    //     context.fillStyle = "red";
    //     context.fill();
    //     break;
    //   default:
    //     break;
}
  

// Called when a WebSocket error occurs
function onError(evt) {
    console.log("ERROR: " + evt.data);
}

// Sends a message to the server (and prints it to the console)
function doSend(message) {
    console.log("Sending: " + message);
    websocket.send(message);
}

  // Called whenever the HTML button is pressed
  // function onPress() {
  //   doSend("toggleLED");
  //   doSend("getLEDState");
  //}

// Call the init function as soon as the page loads
window.addEventListener("load", init, false);


