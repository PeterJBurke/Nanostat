// Javascript code to set up a websocket.

var m_url_JS = "ws://nanostat.local:81/";
//var m_url_JS = "ws://192.168.1.44:81/";
// var url = "ws://192.168.4.1:1337/";

var m_websocket;
var m_canvas_JS;
var context;
var dataPlot;
var maxDataPoints = 20; // max points in browser cache
var new_binary_data_is_incoming = false; // if true, reset counters, will recieve 3 binary messages with arrays for current voltage time
var amps_array_has_been_received = false;
var volts_array_has_been_received = false;
var time_array_has_been_received = false;
var amps_array = []; // these have to be global and filled one by one, assume browswer has infinite processing power and memory
var volts_array = []; // these have to be global and filled one by one, assume browswer has infinite processing power and memory
var time_array = []; // these have to be global and filled one by one, assume browswer has infinite processing power and memory

// This is called when the page finishes loading
function init() {

    // Assign page elements to variables
    m_canvas_JS = document.getElementById("m_canvas");

    // create chart:

    // dataPlot = new Chart(document.getElementById("m_canvas"), {
    //     type: 'line',
    //     data: {
    //         labels: [],
    //         datasets: [{
    //             data: [],
    //             label: "Temperature (C)",
    //             borderColor: "#3e95cd",
    //             fill: false
    //         }]
    //     },
    //     options: {
    //         scales: {
    //             y: {
    //                 beginAtZero: true
    //             }
    //         }
    //     }
    // });

    // Connect to WebSocket server
    wsConnect(m_url_JS);
}

// Call this to connect to the WebSocket server
function wsConnect(m_url_JS) {

    // Connect to WebSocket server
    m_url_JS = "ws://" + window.location.hostname + ":81/"
    // console.log(m_url_JS);
    m_websocket = new WebSocket(m_url_JS);
    m_websocket.binaryType = "arraybuffer";

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

// remove excess data from plot
function removeData() {
    dataPlot.data.labels.shift();
    dataPlot.data.datasets[0].data.shift();
}

// add data to plot (through chart object push method...)
function addData(label, data) {
    if (dataPlot.data.labels.length > maxDataPoints) removeData();
    dataPlot.data.labels.push(label);
    dataPlot.data.datasets[0].data.push(data);
    dataPlot.update();
}




// Called when a message is received from the server
function onMessage(evt) {
    console.log("onMessage called");


    if (typeof (evt.data) == "object") { // payload is binary, an ArrayBuffer 
        console.log("OBJECT! parsing....");
        const view = new DataView(evt.data);
        var m_num_points;
        if (amps_array_has_been_received == false) {// this is the amps array
            console.log("this is the amps array");
            m_num_points = evt.data.byteLength / 4;
            console.log(" m_num_points=evt.data.byteLength/4=");
            console.log(m_num_points);
            amps_array.splice(0, amps_array.length); // empty old amps array
            for (i = 0; i < m_num_points; i++) {
                // console.log(i, view.getFloat32(i * 4, true));
                amps_array.push(view.getFloat32(i * 4, true)); // Add element to array
            }
            amps_array_has_been_received = true;
            return;
        }
        if ((amps_array_has_been_received == true) & (volts_array_has_been_received == false)) {// this is the volts array
            console.log("this is the volts array");
            m_num_points = evt.data.byteLength / 2;
            console.log(" m_num_points=evt.data.byteLength/2=");
            console.log(m_num_points);
            volts_array.splice(0, volts_array.length); // empty old amps array
            for (i = 0; i < m_num_points; i++) {
                // console.log(i, view.getInt16(i * 2, true));
                volts_array.push(view.getInt16(i * 2, true)); // Add element to array
            }
            volts_array_has_been_received = true;
            return;
        }
        if ((amps_array_has_been_received == true) & (volts_array_has_been_received == true)) {// this is the time array
            console.log("this is the time array");
            m_num_points = evt.data.byteLength / 4;
            console.log(" m_num_points=evt.data.byteLength/4=");
            console.log(m_num_points);
            time_array.splice(0, time_array.length); // empty old amps array
            for (i = 0; i < m_num_points; i++) {
                // console.log(i, view.getInt32(i * 4, true));
                time_array.push(view.getInt32(i * 4, true)); // Add element to array
            }
            time_array_has_been_received = true;
            new_binary_data_is_incoming = false;
            console.log("Received all 3 arrays binary websocket (I,V,t):");
            console.log(volts_array);
            console.log(amps_array);
            console.log(time_array);
            //  call the graph function now...
            plotGlobalArrays();
            return;

        }


    }


    if (typeof (evt.data) == "string") { // payload is string (JSON probably)
        console.log("STRING! parsing....");


        // Print out our received message
        console.log("Received: " + evt.data);
        var m_json_obj = JSON.parse(evt.data);
        //    console.log(m_json_obj);
        if ('expect_binary_data' in m_json_obj) {// changin in is sweeping status, update indicator on browswer page...
            // do something
            if (m_json_obj.expect_binary_data == true) { // expect binary data
                // set expect data, set amps volts others to invalid, erase them
                new_binary_data_is_incoming = true; // if true, reset counters, will recieve 3 binary messages with arrays for current voltage time
                amps_array_has_been_received = false;
                volts_array_has_been_received = false;
                time_array_has_been_received = false;
                // erase arrays (not sure how in Javascript just yet.... xyz)
                // amps_array; // these have to be global and filled one by one, assume browswer has infinite processing power and memory
                // current_array; // these have to be global and filled one by one, assume browswer has infinite processing power and memory
                // time_array; // these have to be global and filled one by one, assume browswer has infinite processing power and memory
                console.log("(m_json_obj.expect_binary_data == true recieved...");
            }
            if (m_json_obj.expect_binary_data == false) { // mode is sweeping
                // do something
                // actually do nothing, just log to console
                console.log("m_json_obj.expect_binary_data == false recieved...");
            }
        };

        if ('is_sweeping' in m_json_obj) {// changin in is sweeping status, update indicator on browswer page...
            // do something
            if (m_json_obj.is_sweeping == true) { // mode is sweeping
                // do something
                // document.getElementById('sweep_mode_id').innerHTML = "SWEEPING";
                document.getElementById('sweep_mode_id').innerHTML = "<span style=\"color:red\">SWEEPING</span>";
                console.log("need to update indicator to true...");
            }
            if (m_json_obj.is_sweeping == false) { // mode is sweeping
                // do something
                document.getElementById('sweep_mode_id').innerHTML = "<span style=\"color:green\">IDLE</span>";
                console.log("need to update indicator to false...");
            }
        };



        if ('Voltage' in m_json_obj) {// this is the voltamagram (sent as string in JSON over websocket), parse and plot it...
            var m_voltage_array = m_json_obj.Voltage;
            var m_current_array = m_json_obj.Current;
            var m_time_array = m_json_obj.Time;
            console.log(m_voltage_array);
            console.log(m_current_array);
            console.log(m_time_array);


            var trace_IV = {
                x: m_voltage_array,
                y: m_current_array,
                mode: 'markers',
                type: 'scatter'
            };
            var data_IV = [trace_IV];

            var m_IV_layout = {
                // title: 'IV Curve',
                showlegend: false,
                margin: {
                    l: 50,
                    r: 5,
                    b: 50,
                    t: 1,
                    pad: 4
                },
                // title: {
                //     text:'Plot Title',
                //     font: {
                //       family: 'Courier New, monospace',
                //       size: 24
                //     },
                //     xref: 'paper',
                //     x: 0.05,
                //   },
                xaxis: {
                    title: { text: 'Voltage (mV)' }
                },
                yaxis: {
                    title: { text: 'Current (microA)' }
                }
            };


            // Plotly.newPlot('plotly-IV', data_IV, m_IV_layout, {scrollZoom: true}, {editable: true}, {responsive: true});
            Plotly.newPlot('plotly-IV', data_IV, m_IV_layout, { scrollZoom: true, editable: true, responsive: true });


            var trace_IvsTime = {
                x: m_time_array,
                y: m_current_array,
                mode: 'markers',
                type: 'scatter',
                name: "Current"
            };
            var trace_VvsTime = {
                x: m_time_array,
                y: m_voltage_array,
                mode: 'markers',
                yaxis: 'y2',
                type: 'scatter',
                name: "Voltage"
            };

            var data_IVvsTime = [trace_IvsTime, trace_VvsTime];


            var m_2yaxis_layout = {
                margin: {
                    l: 50,
                    r: 5,
                    b: 50,
                    t: 1,
                    pad: 4
                },
                // title: {
                //     text:'Plot Title',
                //     font: {
                //       family: 'Courier New, monospace',
                //       size: 24
                //     },
                //     xref: 'paper',
                //     x: 0.05,
                //   },
                xaxis: {
                    title: { text: 'Time (ms)' }
                },
                yaxis: { title: 'Current (microA)' },
                yaxis2: {
                    title: 'Voltage (mV)',
                    titlefont: { color: 'rgb(148, 103, 189)' },
                    tickfont: { color: 'rgb(148, 103, 189)' },
                    overlaying: 'y',
                    side: 'right'
                }
            };

            Plotly.newPlot('plotly-IvsTime', data_IVvsTime, m_2yaxis_layout, { scrollZoom: true, editable: true, responsive: true });
            // Since data is fresh, might as well force a download for user....
            // if (download button is on) // not yet impelmented
            // xxxyyyzzz       <li><a href="downloadfile">Download</a></li>
            // window.open("http://nanostat.local/downloadfile");
            window.open("/downloadfile");

        };





    }

}

function addData(label, data) {
    if (dataPlot.data.labels.length > maxDataPoints) removeData();
    dataPlot.data.labels.push(label);
    dataPlot.data.datasets[0].data.push(data);
    dataPlot.update();
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

// Slider calls this to set data rate:
function sendDataRate() {
    var dataRate = document.getElementById("dataRateSlider").value;
    m_websocket.send(dataRate);
    dataRate = 1.0 * dataRate;
    document.getElementById("dataRateLabel").innerHTML = "Rate: " + dataRate.toFixed(2) + "Hz";
}


function plotGlobalArrays() {// plot global arrays that have been filled by the websocket data received as binary

    // console.log(volts_array);
    // console.log(amps_array);
    // console.log(time_array);

    var trace_IV = {
        x: volts_array,
        y: amps_array,
        mode: 'markers',
        type: 'scatter'
    };
    var data_IV = [trace_IV];

    var m_IV_layout = {
        // title: 'IV Curve',
        showlegend: false,
        margin: {
            l: 50,
            r: 5,
            b: 50,
            t: 1,
            pad: 4
        },
        xaxis: {
            title: { text: 'Voltage (mV)' }
        },
        yaxis: {
            title: { text: 'Current (microA)' }
        }
    };

    Plotly.newPlot('plotly-IV', data_IV, m_IV_layout, { scrollZoom: true, editable: true, responsive: true });

    var trace_IvsTime = {
        x: time_array,
        y: amps_array,
        mode: 'markers',
        type: 'scatter',
        name: "Current"
    };
    var trace_VvsTime = {
        x: time_array,
        y: volts_array,
        mode: 'markers',
        yaxis: 'y2',
        type: 'scatter',
        name: "Voltage"
    };

    var data_IVvsTime = [trace_IvsTime, trace_VvsTime];

    var m_2yaxis_layout = {
        margin: {
            l: 50,
            r: 5,
            b: 50,
            t: 1,
            pad: 4
        },
        
        xaxis: {
            title: { text: 'Time (ms)' }
        },
        yaxis: { title: 'Current (microA)' },
        yaxis2: {
            title: 'Voltage (mV)',
            titlefont: { color: 'rgb(148, 103, 189)' },
            tickfont: { color: 'rgb(148, 103, 189)' },
            overlaying: 'y',
            side: 'right'
        }
    };

    Plotly.newPlot('plotly-IvsTime', data_IVvsTime, m_2yaxis_layout, { scrollZoom: true, editable: true, responsive: true });
    // Since data is fresh, might as well force a download for user....
    // if (download button is on) // not yet impelmented
    // xxxyyyzzz       <li><a href="downloadfile">Download</a></li>
    // window.open("http://nanostat.local/downloadfile");
    window.open("/downloadfile");





}


// Call the init function as soon as the page loads
window.addEventListener("load", init, false);


