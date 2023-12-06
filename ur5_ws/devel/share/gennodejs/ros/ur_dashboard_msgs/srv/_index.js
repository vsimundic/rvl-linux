
"use strict";

let GetRobotMode = require('./GetRobotMode.js')
let RawRequest = require('./RawRequest.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let GetProgramState = require('./GetProgramState.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let AddToLog = require('./AddToLog.js')
let Load = require('./Load.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let Popup = require('./Popup.js')

module.exports = {
  GetRobotMode: GetRobotMode,
  RawRequest: RawRequest,
  IsProgramRunning: IsProgramRunning,
  GetProgramState: GetProgramState,
  IsProgramSaved: IsProgramSaved,
  AddToLog: AddToLog,
  Load: Load,
  GetSafetyMode: GetSafetyMode,
  GetLoadedProgram: GetLoadedProgram,
  IsInRemoteControl: IsInRemoteControl,
  Popup: Popup,
};
