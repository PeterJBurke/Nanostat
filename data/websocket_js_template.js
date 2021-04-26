// Javascript code to set up a websocket for the LMP91000 control panel.
// see https://www.youtube.com/watch?v=mkXsmCgvy0k

var m_websocket;
var m_url_JS = "ws://nanostat.local:81/";

// This is called when the page finishes loading
function init() {
    wsConnect(m_url_JS);   // Connect to WebSocket server

}

// Call this to connect to the WebSocket server
function wsConnect(m_url_JS) {

    // Connect to WebSocket server
    m_url_JS = "ws://" + window.location.hostname + ":81/"
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

}

// Called when the WebSocket connection is closed
function onClose(evt) {

    // Log disconnection state
    console.log("Disconnected");



    // Try to reconnect after a few seconds
    setTimeout(function () { wsConnect(m_url_JS) }, 2000);
}



// Called when a message is received from the server
function onMessage(evt) {

    // Print out our received message
    console.log("Received: " + evt.data);


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



// Call the init function as soon as the page loads
window.addEventListener("load", init, false);


