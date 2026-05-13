
"use strict";

let HumanSkeleton = require('./HumanSkeleton.js');
let Histogram = require('./Histogram.js');
let PlotData = require('./PlotData.js');
let HeightmapConfig = require('./HeightmapConfig.js');
let HistogramWithRange = require('./HistogramWithRange.js');
let TrackingStatus = require('./TrackingStatus.js');
let ContactSensor = require('./ContactSensor.js');
let ParallelEdge = require('./ParallelEdge.js');
let Torus = require('./Torus.js');
let ICPResult = require('./ICPResult.js');
let SegmentArray = require('./SegmentArray.js');
let SparseOccupancyGrid = require('./SparseOccupancyGrid.js');
let TimeRange = require('./TimeRange.js');
let ClusterPointIndices = require('./ClusterPointIndices.js');
let ColorHistogramArray = require('./ColorHistogramArray.js');
let PolygonArray = require('./PolygonArray.js');
let SnapItRequest = require('./SnapItRequest.js');
let RotatedRect = require('./RotatedRect.js');
let Segment = require('./Segment.js');
let PlotDataArray = require('./PlotDataArray.js');
let WeightedPoseArray = require('./WeightedPoseArray.js');
let PosedCameraInfo = require('./PosedCameraInfo.js');
let ImageDifferenceValue = require('./ImageDifferenceValue.js');
let ParallelEdgeArray = require('./ParallelEdgeArray.js');
let Accuracy = require('./Accuracy.js');
let Int32Stamped = require('./Int32Stamped.js');
let ClassificationResult = require('./ClassificationResult.js');
let Circle2D = require('./Circle2D.js');
let Object = require('./Object.js');
let Rect = require('./Rect.js');
let RectArray = require('./RectArray.js');
let SegmentStamped = require('./SegmentStamped.js');
let RotatedRectStamped = require('./RotatedRectStamped.js');
let Circle2DArray = require('./Circle2DArray.js');
let SimpleHandle = require('./SimpleHandle.js');
let TorusArray = require('./TorusArray.js');
let PointsArray = require('./PointsArray.js');
let SparseOccupancyGridColumn = require('./SparseOccupancyGridColumn.js');
let SlicedPointCloud = require('./SlicedPointCloud.js');
let BoolStamped = require('./BoolStamped.js');
let BoundingBox = require('./BoundingBox.js');
let SparseOccupancyGridCell = require('./SparseOccupancyGridCell.js');
let HistogramWithRangeBin = require('./HistogramWithRangeBin.js');
let ContactSensorArray = require('./ContactSensorArray.js');
let PeoplePoseArray = require('./PeoplePoseArray.js');
let LabelArray = require('./LabelArray.js');
let SparseImage = require('./SparseImage.js');
let Label = require('./Label.js');
let SparseOccupancyGridArray = require('./SparseOccupancyGridArray.js');
let HumanSkeletonArray = require('./HumanSkeletonArray.js');
let TrackerStatus = require('./TrackerStatus.js');
let BoundingBoxArrayWithCameraInfo = require('./BoundingBoxArrayWithCameraInfo.js');
let SimpleOccupancyGrid = require('./SimpleOccupancyGrid.js');
let Line = require('./Line.js');
let ModelCoefficientsArray = require('./ModelCoefficientsArray.js');
let ObjectArray = require('./ObjectArray.js');
let SimpleOccupancyGridArray = require('./SimpleOccupancyGridArray.js');
let LineArray = require('./LineArray.js');
let HistogramWithRangeArray = require('./HistogramWithRangeArray.js');
let VectorArray = require('./VectorArray.js');
let BoundingBoxArray = require('./BoundingBoxArray.js');
let ColorHistogram = require('./ColorHistogram.js');
let BoundingBoxMovement = require('./BoundingBoxMovement.js');
let DepthCalibrationParameter = require('./DepthCalibrationParameter.js');
let PeoplePose = require('./PeoplePose.js');
let DepthErrorResult = require('./DepthErrorResult.js');
let Spectrum = require('./Spectrum.js');

module.exports = {
  HumanSkeleton: HumanSkeleton,
  Histogram: Histogram,
  PlotData: PlotData,
  HeightmapConfig: HeightmapConfig,
  HistogramWithRange: HistogramWithRange,
  TrackingStatus: TrackingStatus,
  ContactSensor: ContactSensor,
  ParallelEdge: ParallelEdge,
  Torus: Torus,
  ICPResult: ICPResult,
  SegmentArray: SegmentArray,
  SparseOccupancyGrid: SparseOccupancyGrid,
  TimeRange: TimeRange,
  ClusterPointIndices: ClusterPointIndices,
  ColorHistogramArray: ColorHistogramArray,
  PolygonArray: PolygonArray,
  SnapItRequest: SnapItRequest,
  RotatedRect: RotatedRect,
  Segment: Segment,
  PlotDataArray: PlotDataArray,
  WeightedPoseArray: WeightedPoseArray,
  PosedCameraInfo: PosedCameraInfo,
  ImageDifferenceValue: ImageDifferenceValue,
  ParallelEdgeArray: ParallelEdgeArray,
  Accuracy: Accuracy,
  Int32Stamped: Int32Stamped,
  ClassificationResult: ClassificationResult,
  Circle2D: Circle2D,
  Object: Object,
  Rect: Rect,
  RectArray: RectArray,
  SegmentStamped: SegmentStamped,
  RotatedRectStamped: RotatedRectStamped,
  Circle2DArray: Circle2DArray,
  SimpleHandle: SimpleHandle,
  TorusArray: TorusArray,
  PointsArray: PointsArray,
  SparseOccupancyGridColumn: SparseOccupancyGridColumn,
  SlicedPointCloud: SlicedPointCloud,
  BoolStamped: BoolStamped,
  BoundingBox: BoundingBox,
  SparseOccupancyGridCell: SparseOccupancyGridCell,
  HistogramWithRangeBin: HistogramWithRangeBin,
  ContactSensorArray: ContactSensorArray,
  PeoplePoseArray: PeoplePoseArray,
  LabelArray: LabelArray,
  SparseImage: SparseImage,
  Label: Label,
  SparseOccupancyGridArray: SparseOccupancyGridArray,
  HumanSkeletonArray: HumanSkeletonArray,
  TrackerStatus: TrackerStatus,
  BoundingBoxArrayWithCameraInfo: BoundingBoxArrayWithCameraInfo,
  SimpleOccupancyGrid: SimpleOccupancyGrid,
  Line: Line,
  ModelCoefficientsArray: ModelCoefficientsArray,
  ObjectArray: ObjectArray,
  SimpleOccupancyGridArray: SimpleOccupancyGridArray,
  LineArray: LineArray,
  HistogramWithRangeArray: HistogramWithRangeArray,
  VectorArray: VectorArray,
  BoundingBoxArray: BoundingBoxArray,
  ColorHistogram: ColorHistogram,
  BoundingBoxMovement: BoundingBoxMovement,
  DepthCalibrationParameter: DepthCalibrationParameter,
  PeoplePose: PeoplePose,
  DepthErrorResult: DepthErrorResult,
  Spectrum: Spectrum,
};
