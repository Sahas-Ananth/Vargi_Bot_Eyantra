var jsonDataObject = [];

function RefreshAll() {
	$.getJSON('https://spreadsheets.google.com/feeds/list/1JvXdYoYRmdPyDNdNzlCEjxApYi0O4UX8D39OOYxgt0o/5/public/full?alt=json', function(data) 
		{
			for (var i = 0; i < data.feed.entry.length; ++i) {

				var json_data = {
						"Timestamp":data.feed.entry[i].gsx$timestamp,
						"OrderID": data.feed.entry[i].gsx$orderid.$t,
						"Item": data.feed.entry[i].gsx$item.$t,
						"Priority": data.feed.entry[i].gsx$priority.$t,
						"Quantity": data.feed.entry[i].gsx$quantity.$t,
						"City": data.feed.entry[i].gsx$city.$t,
						"Longitude": parseFloat(data.feed.entry[i].gsx$longitude.$t),
						"Latitude": parseFloat(data.feed.entry[i].gsx$latitude.$t),
						"Dispatched": data.feed.entry[i].gsx$dispatched.$t,
						"Shipped": data.feed.entry[i].gsx$shipped.$t,
						"Order_Date_and_Time": data.feed.entry[i].gsx$orderdateandtime,
						"Dispatch_Date_and_Time": data.feed.entry[i].gsx$dispatchdateandtime,
						"Shipped_Date_and_Time": data.feed.entry[i].gsx$shippeddateandtime,
						"TimeTaken": parseFloat(data.feed.entry[i].gsx$timetaken.$t)
				};
				jsonDataObject.push(json_data);
				TableRefresh();
				ChartRefresh();
				MapRefresh();
			}
		}
	)
}


function TableRefresh() {
	refreshTable();
	setInterval(refreshTable, 1000);
}

function ChartRefresh() {
	refreshChart();
	setInterval(refreshChart, 5000);
}

function MapRefresh() {
	refreshMap();
	setInterval(refreshMap, 5000);
}

function refreshTable() {
	var trHTML = '';
	for (var j = 0; j < jsonDataObject.length; ++j) {
			trHTML += 	'<tr><td>' + jsonDataObject[j].OrderID + 
						'</td><td>' + jsonDataObject[j].Item + 
						'</td><td>' + jsonDataObject[j].Priority + 
						'</td><td>' + jsonDataObject[j].Quantity + 
						'</td><td>'  + jsonDataObject[j].City + 
						'</td></tr>';
	}
	console.log(trHTML);
	$('#tableContent').html(trHTML);
	var trHTML = '';
}

function refreshMap(){
	var container = L.DomUtil.get('map');

	if(container != null){
		container._leaflet_id = null;
	}
	var map=L.map('map', {zoomControl: false}).setView([23.0000, 81.0000], 5);

	map.touchZoom.disable();
	map.doubleClickZoom.disable();
	map.scrollWheelZoom.disable();
	map.boxZoom.disable();
	map.keyboard.disable();

	for (var j = 0; j < jsonDataObject.length; j++) {
			var customColour = "red";
			var pos = L.latLng(parseFloat(jsonDataObject[j].Latitude), parseFloat(jsonDataObject[j].Longitude))

			if (jsonDataObject[j].Dispatched == "Yes" && jsonDataObject[j].Shipped == ""){
					customColour = "orange";
			}
			else if ((jsonDataObject[j].Dispatched == "Yes" && jsonDataObject[j].Shipped == "Yes")){
					customColour = "green";
			}
			
			var customMarker = L.AwesomeMarkers.icon({
					markerColor: customColour
			});

			var marker = L.marker(pos, {icon: customMarker});
			marker.bindPopup(jsonDataObject[j].City, {
							autoClose: false
			});
			map.addLayer(marker);
			marker.on('click', onClick_Marker)
			// Attach the corresponding JSON data to your marker:
			marker.myJsonData =jsonDataObject[j];

			function onClick_Marker(e) {
							var marker = e.target;
							popup = L.popup()
							.setLatLng(marker.getLatLng())
							.setContent("Order ID: " + marker.myJsonData.OrderID + " || Item: " +   marker.myJsonData.Item)
							.openOn(map);
					}

			L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
							attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
			}).addTo(map); 
							

	}
}

google.charts.load("current", {packages:['corechart']});
google.charts.setOnLoadCallback(refreshChart);

function refreshChart(){
	var graph_arr = [['Order ID', 'Time Taken', { role: 'style' }]];
	var bar_color = [];
	for (var j in jsonDataObject) {
		if(jsonDataObject[j].Priority == 'HP'){
			var color =  '#FF0000';
			}
		else if(jsonDataObject[j].Priority == 'MP'){
			var color =  '#FFFF00';
			}
		else if(jsonDataObject[j].Priority == 'LP'){
			var color =  '#00FF00';
			}
		bar_color.push(color)
	}

	// Converting Json Object to JavaScript Array
	for(var j in jsonDataObject){
			graph_arr.push([jsonDataObject[j].OrderID,jsonDataObject[j].TimeTaken, bar_color[j]]);
	}
	var graphArray_Final = google.visualization.arrayToDataTable(graph_arr);

	var data = new google.visualization.DataView(graphArray_Final); 

	var options = {
		title: 'Time Taken for items to be Shipped',
		hAxis: { title: 'Order ID'},
		vAxis: { title: 'Time Taken (s)'},
		legend: { position: "none" },
	};
	var chart = new google.visualization.ColumnChart(document.getElementById('column_chart'));
	chart.draw(data, options);
}
