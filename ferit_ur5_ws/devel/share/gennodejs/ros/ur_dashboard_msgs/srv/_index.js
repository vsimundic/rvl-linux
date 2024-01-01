
"use strict";

let GetLoadedProgram = require('./GetLoadedProgram.js')
let GetProgramState = require('./GetProgramState.js')
let Popup = require('./Popup.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let AddToLog = require('./AddToLog.js')
let RawRequest = require('./RawRequest.js')
let Load = require('./Load.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let GetRobotMode = require('./GetRobotMode.js')
let IsProgramSaved = require('./IsProgramSaved.js')

module.exports = {
  GetLoadedProgram: GetLoadedProgram,
  GetProgramState: GetProgramState,
  Popup: Popup,
  GetSafetyMode: GetSafetyMode,
  IsInRemoteControl: IsInRemoteControl,
  AddToLog: AddToLog,
  RawRequest: RawRequest,
  Load: Load,
  IsProgramRunning: IsProgramRunning,
  GetRobotMode: GetRobotMode,
  IsProgramSaved: IsProgramSaved,
};
