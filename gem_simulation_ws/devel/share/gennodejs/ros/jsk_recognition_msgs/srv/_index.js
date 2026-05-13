
"use strict";

let SwitchTopic = require('./SwitchTopic.js')
let SnapFootstep = require('./SnapFootstep.js')
let ICPAlign = require('./ICPAlign.js')
let SetDepthCalibrationParameter = require('./SetDepthCalibrationParameter.js')
let NonMaximumSuppression = require('./NonMaximumSuppression.js')
let CheckCollision = require('./CheckCollision.js')
let SetLabels = require('./SetLabels.js')
let CallPolygon = require('./CallPolygon.js')
let RobotPickupReleasePoint = require('./RobotPickupReleasePoint.js')
let TowerRobotMoveCommand = require('./TowerRobotMoveCommand.js')
let WhiteBalancePoints = require('./WhiteBalancePoints.js')
let ICPAlignWithBox = require('./ICPAlignWithBox.js')
let SetPointCloud2 = require('./SetPointCloud2.js')
let EnvironmentLock = require('./EnvironmentLock.js')
let CheckCircle = require('./CheckCircle.js')
let WhiteBalance = require('./WhiteBalance.js')
let UpdateOffset = require('./UpdateOffset.js')
let TransformScreenpoint = require('./TransformScreenpoint.js')
let SetTemplate = require('./SetTemplate.js')
let TowerPickUp = require('./TowerPickUp.js')
let PolygonOnEnvironment = require('./PolygonOnEnvironment.js')
let SaveMesh = require('./SaveMesh.js')
let EuclideanSegment = require('./EuclideanSegment.js')
let CallSnapIt = require('./CallSnapIt.js')

module.exports = {
  SwitchTopic: SwitchTopic,
  SnapFootstep: SnapFootstep,
  ICPAlign: ICPAlign,
  SetDepthCalibrationParameter: SetDepthCalibrationParameter,
  NonMaximumSuppression: NonMaximumSuppression,
  CheckCollision: CheckCollision,
  SetLabels: SetLabels,
  CallPolygon: CallPolygon,
  RobotPickupReleasePoint: RobotPickupReleasePoint,
  TowerRobotMoveCommand: TowerRobotMoveCommand,
  WhiteBalancePoints: WhiteBalancePoints,
  ICPAlignWithBox: ICPAlignWithBox,
  SetPointCloud2: SetPointCloud2,
  EnvironmentLock: EnvironmentLock,
  CheckCircle: CheckCircle,
  WhiteBalance: WhiteBalance,
  UpdateOffset: UpdateOffset,
  TransformScreenpoint: TransformScreenpoint,
  SetTemplate: SetTemplate,
  TowerPickUp: TowerPickUp,
  PolygonOnEnvironment: PolygonOnEnvironment,
  SaveMesh: SaveMesh,
  EuclideanSegment: EuclideanSegment,
  CallSnapIt: CallSnapIt,
};
