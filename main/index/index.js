

// init chart variables
// var mylabels = [1, 2, 3, 4, 5, 6];
// var mydata = [0, 0, 10, 10, 20, 10];
// var mydata2 = [10, 10, 1, 1, 2, 10];
// // chart variables
// var labelsSensors = [1, 2, 3, 4, 5, 6];
// var dataSensor1 = [1, 1, 1, 1, 1, 1];
// var dataSensor2 = [20, 20, 20, 20, 20, 20];

// ==========================================================================
// create chart with standard values
//
// var ctx = document.getElementById('myChart').getContext('2d');
// // define size of chart (not in .css)
// ctx.canvas.parentNode.style.width = "1000px";
// ctx.canvas.parentNode.style.height = "350px";
// // create chart and define options
// var myChart = new Chart(ctx, {
//     type: 'line',
//     data: {
//         labels: mylabels,
//         datasets: [{
//             label: 'Yarn Tension 1',
//             fill: false,
//             data: mydata,
//             borderWidth: 5,
//             borderColor: 'rgba(000, 139, 000, 1)'
//         },{
//             label: 'Yarn Tension 2',
//             fill: false,
//             data: mydata2,
//             borderWidth: 5,
//             borderColor: 'rgba(100, 250, 050, 1)'
//         }]
//     },
//     options: {
//         elements: {
//             point: {
//                 radius: 0
//             }
//         },
//         responsive: true,
//         maintainAspectRatio: false,
//         animation: {
//             duration: 0
//         },
//         scales: {
//             yAxes: [{
//                 ticks: {
//                     min: 0,
//                     max: 50,
//                     stepSize: 5,
//                     beginAtZero: true
//                 },
//                 scaleLabel:{
//                     display: true,
//                     labelString: 'tension [cN]',
//                     fontSize: 15
//                 }
//             }],
//             xAxes: [{
//                 ticks: {
//                     min: 0,
//                     max: 360,
//                     stepSize: 10,
//                     beginAtZero: true,
//                     suggestedMax: 100
//                 },
//                 scaleLabel:{
//                     display: true,
//                     labelString: 'machine angle [°]',
//                     fontSize: 15
//                 }
//             }]
//         }
//     }
// });
// //

// global elements
var dropDown = document.getElementById("dropDownColors");

var myresponse;
var myresult

var myArrayResponse;

// =========================================================================================
function init(){
    console.log("Init called");
    // document.getElementById("outputBTSR1").value = 15;
    // document.getElementById("outputBTSR2").value = 14;

    document.getElementById("buttonSingleColor").addEventListener("click", changeToSingleColor);
    document.getElementById("buttonBlinkLine").addEventListener("click", changeToBlinkLine);
    document.getElementById("buttonBlend").addEventListener("click", changeToBlend);
    

    // get my dropdown menu
    //var e = document.getElementById("cars");
    // get selected value
    //e.options[e.selectedIndex].value;

    // document.getElementById("setpointButton1").addEventListener("click", function(){sendhdrivesetpoint(1)});
    // document.getElementById("setpointButton2").addEventListener("click", function(){sendhdrivesetpoint(2)});

    //document.getElementById("iosTestButton").addEventListener("click", testfunction);


    //document.getElementById("getarraybutton").addEventListener("click", getTensionMean);

    // document.getElementById("changeToSensor1button").addEventListener("click", function(){switchSensorView(1)});
    // document.getElementById("changeToSensor2button").addEventListener("click", function(){switchSensorView(2)});
    // document.getElementById("changeToSensorAllbutton").addEventListener("click", function(){switchSensorView(3)});
    

    // toggle control mode
    // document.getElementById("manualButton").addEventListener("click", function(){
    //                                                                         sendhdrivesetpoint(1);
    //                                                                         setTimeout(sendhdrivesetpoint(2), 500);
    //                                                                         setTimeout(toggleControlMode(0),500)
    //                                                                         });
    // document.getElementById("autoButton").addEventListener("click", function(){
    //                                                                         sendhdrivesetpoint(1);
    //                                                                         setTimeout(sendhdrivesetpoint(2), 500);
    //                                                                         setTimeout(toggleControlMode(1),500)
    //                                                                         });

    // // ios tests
    // document.getElementById("iosTestText").innerHTML = '---' + 0 + ' ---';
    
    
    //document.getElementById("buttonUpdateGRB").addEventListener("click", sendGRB);

    document.getElementById("InputR").addEventListener("click", function(){dropDown.selectedIndex=0});
    document.getElementById("InputG").addEventListener("click", function(){dropDown.selectedIndex=0});
    document.getElementById("InputB").addEventListener("click", function(){dropDown.selectedIndex=0});

    

    //document.getElementById("dropDownColors").addEventListener("onchange", sendGRB);

    
    
}
//
// function to write number field

// cyclic function (all 100ms)
// read dropdown menu 
// see if value has changed
// if changed, write number in number field

var updateRGBFiled = setInterval(watchRGBSelection, 200);
var currentSelection = "red";
var oldSelection = currentSelection;
dropDown.selectedIndex=1;
var r = 255;
var g = 0;
var b = 0;
var rString ="0";
var gString ="0";
var bString ="0";
var rgb = "ff0000"
var grb = "00ff00"
var grb_old = grb;
// function to watch dropdown Input and handle rgb input fields
function watchRGBSelection(){

    currentSelection = dropDown.options[dropDown.selectedIndex].value;

    if(currentSelection != oldSelection){
        if(currentSelection == "red"){
            r = 255;
            g = 0;
            b = 0;
        }else if(currentSelection == "green"){
            r = 0;
            g = 255;
            b = 0;
        }else if(currentSelection == "blue"){
            r = 0;
            g = 0;
            b = 255;
        }else if(currentSelection == "pink"){
            r = 255;
            g = 20;
            b = 147;
        } // other colors
        else if(currentSelection == "rgb"){
            // here we don't change rgb values to predefined set
            // because we want manual choice. Selection "rgb" is
            // when we click onto rgb fields to change values manualy
        }
        // write rgb fields if we have used the dropdown menu:
        document.getElementById("InputR").value = r;
        document.getElementById("InputG").value = g;
        document.getElementById("InputB").value = b;
    }
    // timeshift
    oldSelection = currentSelection;

    //
    // always check boundaries, cyclic:
    var tempR = document.getElementById("InputR").value;
    var tempG = document.getElementById("InputG").value;
    var tempB = document.getElementById("InputB").value;
    tempR = Math.round(tempR);
    tempG = Math.round(tempG);
    tempB = Math.round(tempB);
    if(tempR > 255){
        tempR = 255;
    }else if(tempR < 0){
        tempR = 0;
    }
    if(tempG > 255){
        tempG = 255;
    }else if(tempG < 0){
        tempG = 0;
    }
    if(tempB > 255){
        tempB = 255;
    }else if(tempB < 0){
        tempB = 0;
    }
    document.getElementById("InputR").value = tempR;
    document.getElementById("InputG").value = tempG;
    document.getElementById("InputB").value = tempB;

    // create strings out of dropdown selection. If f.e. 0f, then we have to
    // add a "0" to the string so we always get 2 characters
    if(tempR <= 15){
        rString = "0" + tempR.toString(16)
    }else{
        rString = tempR.toString(16)
    }
    if(tempG <= 15){
        gString = "0" + tempG.toString(16)
    }else{
        gString = tempG.toString(16)
    }
    if(tempB <= 15){
        bString = "0" + tempB.toString(16)
    }else{
        bString = tempB.toString(16)
    }

    // fill data to send
    rgb = rString + gString + bString;
    grb = gString + rString + bString;
    // if grb value has changed, send to esp
    if(grb != grb_old){
        sendGRB();
    }
    grb_old = grb;
    

}



// ====================================
// FUNKCTION to send rgb (or grb) on click uf update button

// onclick stuff oben im init

// get value in number fields

// send


function sendGRB(){
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    // build string to send (POST)
    var postCommandString = "sendGRB" + grb;
    //
    // ==== get values of manual setpoint =========================
    // var elementIdString = "InputManualSetpoint" + hdrive_ch_nr;
    // var firstString  = '';
    // var tempVar = document.getElementById(elementIdString).value;
    // // round value to nearest integer
    // tempVar = Math.round(tempVar);
    // // check boundries
    // if(tempVar >= 100){
    //     tempVar = 100;
    //     firstString = tempVar;
    // }else if((tempVar < 100) && (tempVar >= 10)){
    //     firstString = "0" + tempVar;
    // }else if((tempVar < 10) && (tempVar >= 0)){
    //     firstString = "00" + tempVar;
    // }else if(tempVar < 0){
    //     tempVar = 0;
    //     firstString = "000";
    // }else{
    //     firstString = tempVar;
    // }
    // // copy to hmi
    // document.getElementById(elementIdString).value = tempVar;
    // //
    // // ==== get values of auto setpoint ========================
    // var elementIdString = "InputAutoSetpoint" + hdrive_ch_nr;
    // var secondString  = '';
    // var tempVar = document.getElementById(elementIdString).value;
    // // round value to nearest integer
    // tempVar = Math.round(tempVar);
    // // check boundries
    // if(tempVar >= 50){
    //     tempVar = 50;
    //     secondString = tempVar;
    // }else if((tempVar < 10) && (tempVar >= 0)){
    //     secondString = "0" + tempVar;
    // }else if(tempVar < 0){
    //     tempVar = 0;
    //     secondString = "00";
    // }else{
    //     secondString = tempVar;
    // }
    // // copy to hmi
    // document.getElementById(elementIdString).value = tempVar;
    // //
    // //var stringToSend = '';
    // //stringToSend = firstString + secondString;

    // xhttp.open("POST", postCommandString + firstString + secondString, true);
    xhttp.open("POST", postCommandString, true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}



// var i = 0;
// function testfunction(){
//     document.getElementById("testoutput").value = i;
//     i++;
//     i++;
// }
// =========================================================================================
// function to update chart
// function plotTensionData(sensors2plot){
//     // update chart first time
//     if(sensors2plot == 1){
//         myChart.data.datasets[0].data = dataSensor1;
//         myChart.data.datasets[0].label = 'Yarn Tension 1';
//         myChart.data.datasets[0].borderColor= 'green';

//         myChart.data.datasets[1].data = [];
//         myChart.data.datasets[1].label = [];
//         myChart.data.datasets[1].borderColor= 'white';

//         myChart.data.labels = labelsSensors;

//     }else if(sensors2plot == 2){
//         myChart.data.datasets[0].data = dataSensor2;
//         myChart.data.datasets[0].label = 'Yarn Tension 2';
//         myChart.data.datasets[0].borderColor= 'red';

//         myChart.data.datasets[1].data = [];
//         myChart.data.datasets[1].label = [];
//         myChart.data.datasets[1].borderColor= 'white';

//         myChart.data.labels = labelsSensors;

//     }else if(sensors2plot == 3){
//         myChart.data.datasets[0].data = dataSensor1;
//         myChart.data.datasets[0].label = 'Yarn Tension 1';
//         myChart.data.datasets[0].borderColor= 'green';

//         myChart.data.datasets[1].data = dataSensor2;
//         myChart.data.datasets[1].label = 'Yarn Tension 2';
//         myChart.data.datasets[1].borderColor= 'red';

//         myChart.data.labels = labelsSensors;

//     }
//     myChart.update();
// }
// =========================================================================================
// function to swicth sensor view
// global variable for which Sensors should be plotted
// var sensorChoice = 3;
// function switchSensorView(sensorNr){
//     // change sensor choice to visualize
//     sensorChoice = sensorNr;
//     // update chart first time after manual change of view
//     plotTensionData(sensorChoice);
// }
// // =========================================================================================
// var updateGetTensionMean = setInterval(getTensionMean, 500);
// var initCounter = 0;
// function getTensionMean(){
//     var xhttp = new XMLHttpRequest();
//     xhttp.onreadystatechange = function() {
//         if (this.readyState == 4 && this.status == 200) {
//             myresponse = new Uint8Array(this.response);

//             // first two bytes = control modes:
//             var controlModeFeedback = myresponse[0];
//             //
//             // next four bytes are tension means:
//             var tensionMean0 = (((myresponse[3] & 0xFF) << 8) | (myresponse[2] & 0xFF));
//             var tensionMean1 = (((myresponse[5] & 0xFF) << 8) | (myresponse[4] & 0xFF));
//             // divide by 100 because sent a such:
//             tensionMean0 = tensionMean0 / 100;
//             tensionMean1 = tensionMean1 / 100;
//             // then round to 1 decimal:
//             tensionMean0 = Math.round(tensionMean0 * 10) / 10;
//             tensionMean1 = Math.round(tensionMean1 * 10) / 10;
//             // next two bytes are feedback %-setpoint (actual closed) positions
//             var feedbackSetpoint0 = myresponse[6];
//             var feedbackSetpoint1 = myresponse[7];
//             // next two bytes are feedback auto setpoint in [cN]
//             var feedbackAutoSetpoint0 = myresponse[8];
//             var feedbackAutoSetpoint1 = myresponse[9];
//             // next two bytes are feedback manual setpoint in [%]
//             var feedbackManualSetpoint0 = myresponse[10];
//             var feedbackManualSetpoint1 = myresponse[11];

//             //
//             // write to hmi
//             document.getElementById("outputBTSR1").value =  tensionMean0;
//             document.getElementById("outputBTSR2").value =  tensionMean1;
//             //
//             // if Manual Mode:
//             if(controlModeFeedback == 0){
//                 document.getElementById("controlModeStatus").innerHTML = "Active Mode: MANUAL";
//                 // change colors according to mode
//                 // active texts:
//                 document.getElementById("InputManualSetpoint1").style.color = "green";
//                 document.getElementById("InputManualSetpoint2").style.color = "green";
//                 document.getElementById("text00").style.color = "green";
//                 document.getElementById("text10").style.color = "green";
//                 // inactive texts:
//                 document.getElementById("InputAutoSetpoint1").style.color = "black";
//                 document.getElementById("InputAutoSetpoint2").style.color = "black";          
//                 document.getElementById("text01").style.color = "black";
//                 document.getElementById("text11").style.color = "black";

//                 //document.getElementById("InputAutoSetpoint1").value = feedbackAutoSetpoint0;
//                 //document.getElementById("InputAutoSetpoint2").value = feedbackAutoSetpoint1;

//             // if Auto Mode:
//             }else if(controlModeFeedback == 1){
//                 document.getElementById("controlModeStatus").innerHTML = "Active Mode: AUTO";
//                 // overwrite actual setpoint position in Auto Mode
//                 document.getElementById("InputManualSetpoint1").value = feedbackSetpoint0;
//                 document.getElementById("InputManualSetpoint2").value = feedbackSetpoint1;
//                 // active texts:
//                 document.getElementById("InputAutoSetpoint1").style.color = "green";
//                 document.getElementById("InputAutoSetpoint2").style.color = "green";
//                 document.getElementById("text01").style.color = "green";
//                 document.getElementById("text11").style.color = "green";
//                 // inactive texts:
//                 document.getElementById("InputManualSetpoint1").style.color = "black";
//                 document.getElementById("InputManualSetpoint2").style.color = "black";
//                 document.getElementById("text00").style.color = "black";
//                 document.getElementById("text10").style.color = "black";
//             }
//             // for first two/three cycles, just load init values
//             if(initCounter<2){
//                 document.getElementById("InputManualSetpoint1").value = feedbackManualSetpoint0;
//                 document.getElementById("InputManualSetpoint2").value = feedbackManualSetpoint1;
//                 document.getElementById("InputAutoSetpoint1").value = feedbackAutoSetpoint0;
//                 document.getElementById("InputAutoSetpoint2").value = feedbackAutoSetpoint1;
//                 initCounter++;
//             }
//         }
//     };

    

//     xhttp.open("GET", "getData", true);
//     xhttp.responseType = "arraybuffer";
//     xhttp.send();


// }
// // =========================================================================================
// var cycleUntilUpdate = 0;
// var updateArray = setInterval(getArray, 200);
// function getArray(){
//     var xhttp = new XMLHttpRequest();
//     xhttp.onreadystatechange = function() {
//         if (this.readyState == 4 && this.status == 200) {
//             myArrayResponse = new Uint8Array(this.response);

//             //myresult = (((myresponse[1] & 0xFF) << 8) | (myresponse[0] & 0xFF));
            
//             // clear label vector and data vectors
//             mylabels = [];
//             mydata   = [];
//             mydata2   = [];

//             labelsSensors = [];       

//             // get length of current, valid array of data (1 measurement)
//             var numberOfDataPoints = myArrayResponse.length/2;
//             // create time axis in degrees
//             var i;
//             for (i = 0; i < numberOfDataPoints; i++) { 
//                 mylabels[i] = i;
//                 labelsSensors[i] = Math.round(i * 360 / numberOfDataPoints);
//             } 
//             // set up temp arrays
//             var tempData1 = [];
//             var tempData2 = [];
//             var tempInt1  = 0;
//             var tempInt2  = 0;
//             // fill these arrays with received data and sum up total ("integral")
//             for (i = 0; i < numberOfDataPoints; i++) { 
//                 tempData1[i] = myArrayResponse[i];
//                 tempInt1 += myArrayResponse[i];
//                 tempData2[i] = myArrayResponse[i+numberOfDataPoints];
//                 tempInt2 += myArrayResponse[i+numberOfDataPoints];
//             }
//             // calc. integral, Ts = 2ms (Frequency of data Acquisition, dataAcq.c)
//             tempInt1 = tempInt1 * 0.002;
//             tempInt2 = tempInt2 * 0.002;
            
//             // check if datasets are valid, if so, copy!
//             // else they will stay the same
//             // Threshold of 2.0 same as in dataAcq.c
//             if(tempInt1 >= 2.0){
//                 dataSensor1 = [];
//                 dataSensor1 = tempData1;
//             }
//             if(tempInt2 >= 2.0){
//                 dataSensor2 = [];
//                 dataSensor2 = tempData2;
//             }
//             // replot data depending on global sensor choice

//             plotTensionData(sensorChoice);

//         }
//     };
//     xhttp.open("GET", "getArray", true);
//     xhttp.responseType = "arraybuffer";
//     xhttp.send();  

// }
// // =========================================================================================
// function disableHdrives(){
//     var xhttp = new XMLHttpRequest();
//     xhttp.onreadystatechange = function() {
//         if (this.readyState == 4 && this.status == 200) {
//             // do nothing with response
//             ;
//         }
//     };
//     xhttp.open("POST", "disableHdrives", true);
//     xhttp.responseType = "arraybuffer";
//     xhttp.send();
// }
// // =========================================================================================
// function enableHdrives(){
//     var xhttp = new XMLHttpRequest();
//     xhttp.onreadystatechange = function() {
//         if (this.readyState == 4 && this.status == 200) {
//             // do nothing with response
//             ;
//         }
//     };
//     xhttp.open("POST", "enableHdrives", true);
//     xhttp.responseType = "arraybuffer";
//     xhttp.send();
// }
// // =========================================================================================
// function sendhdrivesetpoint(hdrive_ch_nr){
//     var xhttp = new XMLHttpRequest();
//     xhttp.onreadystatechange = function() {
//         if (this.readyState == 4 && this.status == 200) {
//             // do nothing with response
//             ;
//         }
//     };
//     // build string to send (POST)
//     var postCommandString = "sendhdrive" + hdrive_ch_nr + "setpoint";
//     //
//     // ==== get values of manual setpoint =========================
//     var elementIdString = "InputManualSetpoint" + hdrive_ch_nr;
//     var firstString  = '';
//     var tempVar = document.getElementById(elementIdString).value;
//     // round value to nearest integer
//     tempVar = Math.round(tempVar);
//     // check boundries
//     if(tempVar >= 100){
//         tempVar = 100;
//         firstString = tempVar;
//     }else if((tempVar < 100) && (tempVar >= 10)){
//         firstString = "0" + tempVar;
//     }else if((tempVar < 10) && (tempVar >= 0)){
//         firstString = "00" + tempVar;
//     }else if(tempVar < 0){
//         tempVar = 0;
//         firstString = "000";
//     }else{
//         firstString = tempVar;
//     }
//     // copy to hmi
//     document.getElementById(elementIdString).value = tempVar;
//     //
//     // ==== get values of auto setpoint ========================
//     var elementIdString = "InputAutoSetpoint" + hdrive_ch_nr;
//     var secondString  = '';
//     var tempVar = document.getElementById(elementIdString).value;
//     // round value to nearest integer
//     tempVar = Math.round(tempVar);
//     // check boundries
//     if(tempVar >= 50){
//         tempVar = 50;
//         secondString = tempVar;
//     }else if((tempVar < 10) && (tempVar >= 0)){
//         secondString = "0" + tempVar;
//     }else if(tempVar < 0){
//         tempVar = 0;
//         secondString = "00";
//     }else{
//         secondString = tempVar;
//     }
//     // copy to hmi
//     document.getElementById(elementIdString).value = tempVar;
//     //
//     //var stringToSend = '';
//     //stringToSend = firstString + secondString;

//     xhttp.open("POST", postCommandString + firstString + secondString, true);
//     xhttp.responseType = "arraybuffer";
//     xhttp.send();
// }
// // =========================================================================================
// function toggleControlMode(mode){
//     // first, send setpoints
//     //sendhdrivesetpoint(1);
//     // maybe delay here?
//     //sendhdrivesetpoint(2);
//     // then request change of mode
//     var xhttp = new XMLHttpRequest();
//     xhttp.onreadystatechange = function() {
//         if (this.readyState == 4 && this.status == 200) {
//             // do nothing with response
//             ;
//         }
//     };
//     //
//     var stringToSend = "controlMode" + mode;
//     //
//     xhttp.open("POST", stringToSend, true);
//     xhttp.responseType = "arraybuffer";
//     xhttp.send();
//     //if(mode == 0){
//     //    document.getElementById("controlModeStatus").innerHTML = "Active Mode: MANUAL";
//     //}else if(mode == 1){
//     //    document.getElementById("controlModeStatus").innerHTML = "Active Mode: AUTO";
//     //}
// }


// =========================================================================================
function changeToSingleColor(){
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    xhttp.open("POST", "changeToSingleColor", true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
function changeToBlinkLine(){
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    xhttp.open("POST", "changeToBlinkLine", true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
function changeToBlend(){
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    xhttp.open("POST", "changeToBlend", true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}




// =========================================================================================
// ios Tests
// var iosTestVar = 0;
// //var updateIosFunction = setInterval(iosFunction, 2000);
function iosFunction(){
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            myresponse = new Uint8Array(this.response);


            // --------------
            // iosTestVar++;
            // document.getElementById("iosTestText").innerText = iosTestVar;
            // --------------


            //var tensionMean0 = (((myresponse[1] & 0xFF) << 8) | (myresponse[0] & 0xFF));
            //var tensionMean1 = (((myresponse[3] & 0xFF) << 8) | (myresponse[2] & 0xFF));
            
            //tensionMean0 = tensionMean0 / 100;
            //tensionMean1 = tensionMean1 / 100;
            


            //myresult = (((myresponse[1] & 0xFF) << 8) | (myresponse[0] & 0xFF));

            //myresult = myresult / 100;

            //document.getElementById("outputBTSR1").innerHTML =  '---' + tensionMean0 + ' cN ---';
            //document.getElementById("outputBTSR2").innerHTML =  '---' + tensionMean1 + ' cN ---';
            
            //tension0 = tensionMean0;
            //tension1 = tensionMean1;

        }
    };

    xhttp.open("GET", "iosTest", true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();


}



