// Shorthand for $( document ).ready()
$(function() {

  $('#sliderTemperature').on('input change', function(){
      $("label[for='labelSetTemperature']").text(parseInt(this.value) +"°C");
    });

  $('#sliderHeaterTemperature').on('input change', function(){
      $("label[for='labelSetHeaterTemperature']").text( parseInt(this.value) +"°C");
    });

  $('#sliderFanSpeed').on('input change', function(){
      $("label[for='labelSetFanSpeed']").text(parseInt(this.value) +"%");
    });

  $('#btnSet').on('click', function(){
    var temperature = $("#sliderTemperature").val();
    var heater      = $("#sliderHeaterTemperature").val();
    var fanspeed    = $("#sliderFanSpeed").val();
    console.log("Set command sent");

    // Send
    $.ajax({
      url: "/set",
      type: "get", //send it through get method
      data: {
        temperature : temperature,
        heater      : heater,
        fanspeed    : fanspeed,
      },
      success: function(response) {
        //Do Something
        console.log(response);
        update_status(false);
      },
      error: function(xhr) {
        //Do Something to handle error
        console.log( xhr );
      }
    });
  });

  $('#btnTurnOff').on('click', function(){
    console.log("Turn off command sent");
    // Send
    $.ajax({
      url: "/off",
      type: "get", //send it through get method
      success: function(response) {
        console.log(response);
        update_status(false);
      },
      error: function(xhr) {
        //Do Something to handle error
        console.log( xhr );
      }
    });
  });
  $('#btnReboot').on('click', function(){
      console.log("Reboot command sent");

    // Send
    $.ajax({
      url: "/reboot",
      type: "get", //send it through get method
      data: { },
      success: function(response) {
        //Do Something
        console.log("Response received");
        console.log(response);
        update_status(false);
      },
      error: function(xhr) {
        //Do Something to handle error
        console.log( xhr );
      }
    });
  });

  // Finally update status
  update_status(true);
});


function update_status(rearm) {
  var ajaxTime= new Date().getTime();

  $.ajax({
  url: "/status",
  type: "get", //send it through get method
  success: function(response) {
    console.log(response);

    var res = jQuery.parseJSON(response);

    $('#status-temp-target').text(res.target_temp_in);
    $('#status-fan-speed').text(res.fan_speed);
    $('#status-temp-inside').text(res.temp_in.toFixed(2));
    $('#status-temp-inside2').text(res.temp_in.toFixed(2));
    $('#status-temp-inside-target').text(res.target_temp_in.toFixed(2));
    $('#status-temp-heater').text(res.temp_heater.toFixed(2));
    $('#status-temp-heater-max').text(res.max_temp_heater.toFixed(2));
    $('#status-humidity-inside').text(res.humid_in.toFixed(2));
    $('#status-temp-outside').text(res.temp_out.toFixed(2));
    $('#status-humidity-outside').text(res.humid_out.toFixed(2));

    // Homed status
    if(res.status == 1) {
      $('#status-box').text('Heater is on').addClass('badge-danger').removeClass('badge-secondary').removeClass('badge-success');
      $('#btnSet').prop('disabled', false);
      $('#btnSet').removeClass('disabled');
    }
    else {
      $('#status-box').text('Heater is off').addClass('badge-success').removeClass('badge-secondary').removeClass('badge-danger');
      $('#btnSet').addClass('disabled');
    }

  },
  error: function(xhr) {
    $('#status-box').text('Dry box is unreachable').addClass('badge-secondary').removeClass('badge-success').removeClass('badge-danger');
  }
  }).done(function () {
    var totalTime = new Date().getTime()-ajaxTime;
    console.log( "totalTime: " + totalTime );
    // Here I want to get the how long it took to load some.php and use it further
  });;

  if(rearm) {
    rearm_status();
  }
}

function rearm_status() {
  setTimeout(function () {
    update_status(true);
  }, 2000);
}

