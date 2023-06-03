
"use strict";

let GetChargerByName = require('./GetChargerByName.js')
let GetNumOfWaypoints = require('./GetNumOfWaypoints.js')
let GetWaypointByName = require('./GetWaypointByName.js')
let GetWaypointByIndex = require('./GetWaypointByIndex.js')
let AddNewWaypoint = require('./AddNewWaypoint.js')
let SaveWaypoints = require('./SaveWaypoints.js')

module.exports = {
  GetChargerByName: GetChargerByName,
  GetNumOfWaypoints: GetNumOfWaypoints,
  GetWaypointByName: GetWaypointByName,
  GetWaypointByIndex: GetWaypointByIndex,
  AddNewWaypoint: AddNewWaypoint,
  SaveWaypoints: SaveWaypoints,
};
