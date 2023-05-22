
"use strict";

let AddToLog = require('./AddToLog.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let GetProgramState = require('./GetProgramState.js')
let GetRobotMode = require('./GetRobotMode.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let Load = require('./Load.js')
let Popup = require('./Popup.js')
let RawRequest = require('./RawRequest.js')

module.exports = {
  AddToLog: AddToLog,
  GetLoadedProgram: GetLoadedProgram,
  GetProgramState: GetProgramState,
  GetRobotMode: GetRobotMode,
  GetSafetyMode: GetSafetyMode,
  IsInRemoteControl: IsInRemoteControl,
  IsProgramRunning: IsProgramRunning,
  IsProgramSaved: IsProgramSaved,
  Load: Load,
  Popup: Popup,
  RawRequest: RawRequest,
};
