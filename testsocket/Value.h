#ifndef VALUE_H
#define VALUE_H


// **<Connection>**
//connection's enum
enum CONNECTIONState{
  ONLINE,
  OFFLINE,
  CONNECTIONBROKEN
};

// connection main
struct Connection{
  int headerId=0;
  String timestamp;
  String version;
  String manufacturer;
  String serialNumber;
  CONNECTIONState connectionState;
};

//************************************

//********<Order Message>********                // The message schema to communicate orders from master control to the AGV
// order's enum
enum AGVblockingType{
  NONE,                                          // Cho phép lái xe và các hành động khác
  SOFT,                                          // Cho phép các hành động khác, nhưng không phải lái xe
  HARD                                           // Là hành động duy nhất được phép tại thời điểm đó.
};

enum AGVorientationType{
  GLOBAL,                                        // Liên quan đến hệ tọa độ bản đồ cụ thể toàn cầu
  TANGENTIAL                                     // tiếp xúc với mép. Nếu không được định nghĩa, giá trị mặc định là "TANGENTIAL".
};
// order's object 3
struct nodePositionObject{
  float x;                                       // Dựa trên hệ tọa độ bản đồ, vị trí theo chiều X trên bản đồ. Độ chính xác phụ thuộc vào việc triển khai cụ thể.
  float y;                                       // Dựa trên hệ tọa độ bản đồ, vị trí theo chiều Y trên bản đồ. Độ chính xác phụ thuộc vào việc triển khai cụ thể.
  float theta;                                   // Range[pi,-pi]. Góc quay tuyệt đối của AGV tại điểm (node).Tùy chọn: xe có thể tự lập kế hoạch cho đường đi của mình. Nếu được xác định, AGV phải giả định góc theta tại điểm này.Nếu cạnh trước không cho phép xoay, AGV phải xoay tại điểm này. Nếu cạnh sau có hướng khác nhau được xác định nhưng không cho phép xoay, AGV phải xoay tại điểm này để phù hợp với hướng xoay mong muốn của cạnh trước khi vào cạnh đó.*/
  float allowedDeviationXY;                      // Chỉ ra mức độ chính xác mà một AGV phải đi qua một điểm (node) để được tính là đã đi qua.Nếu bằng 0: không được phép sai lệch (sai lệch bằng 0 có nghĩa là trong phạm vi sai số bình thường của nhà sản xuất AGV).Nếu > 0: sai lệch cho phép - bán kính sai số trong mét. Nếu AGV đi qua một điểm trong phạm vi sai số, điểm được coi là đã được đi qua.
  float allowedDeviationTheta;                   // Range[0,pi].Chỉ ra mức độ lệch góc theta có thể lớn đến đâu.Góc nhỏ nhất chấp nhận được là theta - allowedDeviationTheta và góc lớn nhất chấp nhận được là theta + allowedDeviationTheta.                                    
  String mapId;                                  // Định danh duy nhất của bản đồ mà trong đó vị trí được tham chiếu. Mỗi bản đồ có nguồn gốc toàn cầu cụ thể cho dự án giống nhau về hệ tọa độ.Khi một AGV sử dụng thang máy, ví dụ, từ tầng xuất phát lên tầng đích, nó sẽ biến mất khỏi bản đồ của tầng xuất phát và xuất hiện tại nút thang máy liên quan trên bản đồ của tầng đích.
  String mapDescription;                         // Thông tin bổ sung về bản đồ.
};

struct actionsObject{
  String actionType;                             // Tên của hành động như mô tả trong cột đầu tiên của 'Hành động và Tham số'. Xác định chức năng của hành động.
  String actionId;                               // ID duy nhất để xác định hành động và ánh xạ chúng với actionState trong trạng thái. Gợi ý: Sử dụng UUIDs.
  String actionDescription;                      // Thông tin bổ sung về hành động.
  AGVblockingType blockingType;                  // 

};

// order's object 2
struct nodesObject{
  String nodeId;                                 // Định danh đặc trưng của node
  uint sequenceId;                               // Số để theo dõi chuỗi các node và cạnh trong một đơn đặt hàng và đơn giản hóa cập nhật đơn hàng. Mục đích là phân biệt giữa một node, được đi qua nhiều hơn một lần trong một orderId. Biến sequenceId chạy qua tất cả các node và cạnh của cùng một đơn đặt hàng và được đặt lại khi một orderId mới được phát hành.
  String nodeDescription;                        // Thông tin bổ sung về node.
  bool released;                                 // "true" cho biết rằng node là một phần của base. "false" cho biết rằng node là một phần của horizon.
  nodePositionObject nodePosition;               // Vị trí của node. Tùy chọn cho các loại xe không yêu cầu vị trí của node (ví dụ, xe hướng dẫn bằng đường).
  actionsObject action[10];                     // Mảng các hành động cần được thực hiện tại một node. Mảng trống nếu không có hành động nào được yêu cầu.
};

struct edgesObject{
  String edgeId;                                 // Định danh đặc trưng của cạnh
  uint sequenceId;                               // Số để theo dõi chuỗi các node và cạnh trong một đơn đặt hàng và đơn giản hóa cập nhật đơn hàng. Biến sequenceId chạy qua tất cả các node và cạnh của cùng một đơn đặt hàng và được đặt lại khi một orderId mới được phát hành.
  String edgeDescription;                        // Thông tin bổ xung về edge
  bool released;                                 // "true" cho biết rằng edge là một phần của base. "false" cho biết rằng edge là một phần của horizon.
  String startNodeId;                            // nodeId của startNode.
  String endNodeId;                              // nodeId của endNode.
  float maxSpeed;                                // Tốc độ tối đa được phép trên cạnh. Tốc độ được xác định bởi đo lường nhanh nhất của xe.
  float maxHeight;                               // Chiều cao tối đa được phép của xe, bao gồm cả tải trên cạnh.
  float minHeight;                               // Chiều cao tối thiểu được phép của xe, bao gồm cả tải trên cạnh.
  float orientation;                             // Định hướng của AGV ở mép. Giá trị orientationType xác định liệu nó có phải được hiểu theo hệ tọa độ bản đồ cụ thể toàn cầu hay là theo chiều tiếp xúc với mép. Trong trường hợp được hiểu theo chiều tiếp xúc với mép, 0.0 tương ứng với hướng về phía trước và PI tương ứng với hướng về phía sau. 
  AGVorientationType orientationType;            // Enum {GLOBAL, TANGENTIAL}:
  String direction;                              // Thiết lập hướng tại các nút giao cho các phương tiện được hướng dẫn bằng đường dẫn hoặc dây dẫn, được định nghĩa ban đầu (theo từng phương tiện cụ thể). Ví dụ: trái, phải, thẳng, 433MHz.
  bool rotationAllowed;                          // "true": Cho phép xoay trên mép. "false": Không cho phép xoay trên mép. Tùy chọn: Không có giới hạn nếu không được đặt.
  float maxRotationSpeed;                        // Tốc độ xoay tối đa. Tùy chọn: Không có giới hạn nếu không được đặt.

};

// Order Message main
struct Ordermessage{
  uint32_t headerId;
  String timestamp;
  String version;
  String manufacturer;
  String serialNumber;
  String orderId;
  uint32_t orderUpdateId;                        // orderUpdate identification. Is unique per orderId. If an order update is rejected, this field is to be passed in the rejection message
  uint32_t olderOrderUpdateId=0;
  String zoneSetId;                              // Unique identifier of the zone set that the AGV has to use for navigation or that was used by MC for planning.\nOptional: Some MC systems do not use zones. Some AGVs do not understand zones. Do not add to message if no zones are used
  nodesObject nodes[20];
  edgesObject edges[20];
};

//************************************

// *******<Factsheet>*******
// Factsheet's enum
enum AGVKinematicType {
  DIFF,
  OMNI,
  THREEWHEEL
};

enum AGVClassType {
  FORKLIFT,
  CONVEYOR,
  TUGGER,
  CARRIER
};

enum ArrayOfLocalizationTypes{
  NATURAL,
  REFLECTOR,
  RFID,
  DMC,
  SPOT,
  GRID
};

enum ArrayOfNavigationTypes{
  PHYSICAL_LINDE_GUIDED,
  VIRTUAL_LINE_GUIDED,
  AUTONOMOUS
};
// Factsheet's object
//**2<typeSpecification's Object>2**
struct typeSpecificationObject{
  String seriesName;                            // Free text generalized series name as specified by manufacturer
  String seriesDescription;                     // Free text human readable description of the AGV type series
  AGVKinematicType agvKinematic;                // simplified description of AGV kinematics-type
  AGVClassType agvClass;                        // Simplified description of AGV class
  float maxLoadMass;                            // maximum loadable mass (Kg)
  ArrayOfLocalizationTypes localizationTypes[10];   // simplified description of localization type
  ArrayOfNavigationTypes navigationTypes[10];       // List of path planning types supported by the AGV, sorted by priority
};

//**2<physicalParameters's Object>2**
struct physicalParametersObject{                // <These parameters specify the basic physical properties of the AGV>
  float speedMin;                               // minimal controlled continuous speed of the AGV (m/s)
  float speedMax;                               // maximum speed of the AGV (m/s)
  float accelerationMax;                        // maximum acceleration with maximum load (m/s^2)
  float decelerationMax;                        // maximum deceleration with maximum load (m/s^2)
  float heightMin;                              // minimum height of AGV (m)
  float heightMax;                              // maximum height of AGV (m)
  float width;                                  // width of AGV (m) 
  float length;                                 // length of AGV
};
 
// protocolLimits's Object
struct maxStringLensObject{                     // <maximum lengths of strings>
  int msgLen;                                   // maximum MQTT Message length
  int topicSerialLen;                           // maximum length of serial-number part in MQTT-topics. Affected Parameters: order.serialNumber, instantActions.serialNumber, state.SerialNumber, visualization.serialNumber, connection.serialNumber
  int topicElemLen;                             // maximum length of all other parts in MQTT-topics. Affected parameters: order.timestamp, order.version, order.manufacturer, instantActions.timestamp, instantActions.version, instantActions.manufacturer, state.timestamp, state.version, state.manufacturer, visualization.timestamp, visualization.version, visualization.manufacturer, connection.timestamp, connection.version, connection.manufacturer
  int idLen;                                    // maximum length of ID-Strings. Affected parameters: order.orderId, order.zoneSetId, node.nodeId, nodePosition.mapId, action.actionId, edge.edgeId, edge.startNodeId, edge.endNodeId
  bool idNumericalOnly;                         // If true ID-strings need to contain numerical values only
  int enumLen;                                  // maximum length of ENUM- and Key-Strings. Affected parameters: action.actionType, action.blockingType, edge.direction, actionParameter.key, state.operatingMode, load.loadPosition, load.loadType, actionState.actionStatus, error.errorType, error.errorLevel, errorReference.referenceKey, info.infoType, info.infoLevel, safetyState.eStop, connection.connectionState
  int loadIdLen;                                // maximum length of loadId Strings
};

struct maxArrayLensObject{                      // <maximum lengths of arrays>
  int order_nodes;                              // Maximum number of nodes per order processable by the AGV.
  int order_edges;                              // Maximum number of edges per order processable by the AGV.
  int node_actions;                             // Maximum number of actions per node processable by the AGV.    
  int edge_actions;                             // Maximum number of actions per edge processable by the AGV.
  int actions_actionsParameters;                // Maximum number of parameters per action processable by the AGV.
  int instantActions;                           // Maximum number of instant actions per message processable by the AGV.
  int trajectory_knotVector;                    // Maximum number of knots per trajectory processable by the AGV.
  int trajectory_controlPoints;                 // Maximum number of control points per trajectory processable by the AGV.
  int state_nodeStates;                         // Maximum number of nodeStates sent by the AGV, maximum number of nodes in base of AGV.
  int state_edgeStates;                         // Maximum number of edgeStates sent by the AGV, maximum number of edges in base of AGV.
  int state_loads;                              // Maximum number of load-objects sent by the AGV.
  int state_actionStates;                       // Maximum number of actionStates sent by the AGV.
  int state_errors;                             // Maximum number of errors sent by the AGV in one state-message.
  int state_information;                        // Maximum number of informations sent by the AGV in one state-message.
  int error_errorReferences;                    // Maximum number of error references sent by the AGV for each error.
  int information_infoReferences;                // Maximum number of info references sent by the AGV for each information.
};

struct timingObject{                            // Timing information.
  float minOrderInterval;                       // [s], Minimum interval sending order messages to the AGV.
  float minStateInterval;                       // [s], Minimum interval for sending state- messages.
  float defaultStateInterval;                   // [s], Default interval for sending state- messages, if not defined, the default value from the main document is used.
  float visualizationInterval;                  // [s], Default interval for sending messages on visualization topic.
};

//**2<protocolLimitsObject>2**
struct protocolLimitsObject{
  maxStringLensObject maxStringLens;
  maxArrayLensObject maxArrayLens;
  timingObject timing;
};

// protocolFeatures's Object
//**4<array of optionalParameters's object>4**
enum supportenum{
  SUPPORTED,                                    // optional parameter is supported like specified.
  REQUIRED                                      // optional parameter is required for proper AGV-operation.
};

//**3<optionalParametersObject>3**
struct optionalParametersObject{
  String parameter;                             // Full name of optional parameter, e.g. “order.nodes.nodePosition. allowedDeviationTheta”.
  supportenum support;
  String description;                           // Free-form text: description of optional parameter, e.g.: Reason, why the optional parameter ‘direction’ is necessary for this AGV-type and which values it can contain.The parameter ‘nodeMarker’ must contain unsigned interger-numbers only.NURBS-Support is limited to straight lines and circle segments.
};

//**4<array of agvActions's object>4**
enum actionScopesarrayenum{
  INSTANT,                                      // usable as instantAction.
  NODE,                                         // usable on nodes.
  EDGE                                          // usable on edges.
};

//**4<valueDataTypeenum>4**
enum valueDataTypeenum{
  BOOL,
  NUMBER,
  INTEGER,
  FLOAT,
  STRING,
  OBJECT,
  ARRAY
};

// array of actionParameters's object
//**4<actionParametersObject>4**
struct actionParametersObject{
  String key;                                   // Key-String for Parameter.
  valueDataTypeenum valueDataType;              // Data type of Value, possible data types are: BOOL, NUMBER, INTEGER, FLOAT, STRING, OBJECT, ARRAY.
  String description;                           // Free-form text: description of the parameter.
  bool isOptional;                              // "true": optional parameter.
};

//**3<agvActionsObject>3**
struct agvActionsObject{
  String actionType;                            // Unique actionType corresponding to action.actionType.
  String actionDescription;                     // Free-form text: description of the action.
  actionScopesarrayenum actionScopes[10];       // List of allowed scopes for using this action- type.  
  actionParametersObject actionParameters[10];  // List of parameters If not defined, the action has no parameters
  String resultDescription;                      // Free-form text: description of the resultDescription. 
};

//**2<protocolFeaturesObject>2**
struct protocolFeaturesObject{
  optionalParametersObject optionalParameters[10];
  agvActionsObject agvActions[10];
};

//**4<typeenum>4**
enum typeenum{
  DRIVE,
  CASTER,
  FIXED,
  MECANUM
};

//**4<positionObject>4**
struct positionObject{
  float x;                                      // [m], x-position in AGV-coordinate. system
  float y;                                      // [m], y-position in AGV-coordinate. system
  float theta;                                  // [rad], orientation of wheel in AGV-coordinate system Necessary for fixed wheels.
};

//**3<array of wheelDefinitions's object>3**
struct wheelDefinitionsObject{
  typeenum type;
  bool isActiveDriven;                          // "true": wheel is actively driven (de: angetrieben).
  bool isActiveSteered;                         // "true": wheel is actively steered (de: aktiv gelenkt).
  positionObject position;                      // 
  float diameter;                               // [m], nominal diameter of wheel.
  float width;                                  // [m], nominal width of wheel.
  float centerDisplacement;                     // [m], nominal displacement of the wheel’s center to the rotation point (necessary for caster wheels).If the parameter is not defined, it is assumed to be 0.  
  String constraints;                           // Free-form text: can be used by the manufacturer to define constraints.
};

//**4<array of polygonPoints's object>4**
struct polygonPointsObject{
  float x;                                      // [m], x-position of polygon-point.
  float y;                                      // [m], y-position of polygon-point.
};

//**3<array of envelopes2d's object>3**
struct envelopes2dObject{
  String set;                                   // Name of the envelope curve set.
  polygonPointsObject polygonPoints[10];        // Envelope curve as a x/y-polygon polygon is assumed as closed and must be non-self- intersecting.
  String description;                           // Free-form text: description of envelope curve set.
};

//**3<array of envelopes3d's object>3**
struct envelopes3dObject{
  String set;                                   // Name of the envelope curve set.
  String format;                                // Format of data, e.g., DXF.
  String data;                                  // 3D-envelope curve data, format specified in 'format'.
  String url;                                   // Protocol and url-definition for downloading the 3D-envelope curve data, e.g. ftp://xxx.yyy.com/ac4dgvhoif5tghji.
  String description;                           // Free-form text: description of envelope curve set
};

//**2<agvGeometry's Object>2**
struct agvGeometryObject{
  wheelDefinitionsObject wheelDefinitions[10];  // List of wheels, containing wheel- arrangement and geometry.
  envelopes2dObject envelopes2d[10];            // List of AGV-envelope curves in 2D (german: „Hüllkurven“), e.g., the mechanical envelopes for unloaded and loaded state, the safety fields for different speed cases.
  envelopes3dObject envelopes3d[10];            // List of AGV-envelope curves in 3D (german: „Hüllkurven“).
};

//**3<array of loadSets's Object>3**
struct loadSetsObject{
  String setName;                               // Unique name of the load set, e.g., DEFAULT, SET1, etc.
  String loadType;                              // Type of load, e.g., EPAL, XLT1200, etc.
  String loadPositions[10];                     // List of load positions btw. load handling devices, this load-set is valid for.If this parameter does not exist or is empty, this load-set is valid for all load handling devices on this AGV.
};

//**2<loadSpecification's Object>2**
struct loadSpecificationObject{
  String loadPositions[10];                     // List of load positions / load handling devices.This lists contains the valid values for the parameter “state.loads[].loadPosition” and for the action parameter “lhd” of the actions pick and drop.If this list doesn’t exist or is empty, the AGV has no load handling device.
  loadSetsObject loadSets[10];                  // list of load-sets that can be handled by the AGV

};

//**1<Factsheet main>1**
struct Factsheet{
  int headerId;
  String timestamp;
  String version;
  String manufacturer;
  String serialNumber;
  typeSpecificationObject typeSpecification;
  physicalParametersObject physicalParameters;
  protocolLimitsObject protocolLimits;
  protocolFeaturesObject protocolFeatures;
  agvGeometryObject agvGeometry;
  loadSpecificationObject loadSpecification;
  int localizationParameters;
}; 

//************************************

//*******<State>******** 
// state main
struct State{
  int headerId=0;
  String timestamp ;
  String version ;
  String manufacturer ;
  String serialNumber;
  String orderId ;
  int orderUpdateId  ;                             // orderUpdate identification. Is unique per orderId. If an order update is rejected, this field is to be passed in the rejection message
  String zoneSetId ;   
  String lastNodeId ;
  int lastNodeSequenceId ;
  bool driving ;
  bool paused ;
  bool newBaseRequest ;
};

//************************************

//******<Visualization>******* 
// visualization's object
struct agvPositionObject{
  float x;                                       // 
  float y;                                       //
  float theta;                                   //
  String mapId;                                  //
  bool positionInitialized;                      // True if the AGVs position is initialized, false, if position is not initizalized
  float localizationScore;                       // Localization score for SLAM based vehicles, if the AGV can communicate it. (0<= localizationScore <=1)
  float deviationRange;                          // Value for position deviation range in meters. Can be used if the AGV is able to derive it
};

struct velocityObject{                           // The AGVs velocity in vehicle coordinates
  float vx;
  float vy;
  float omega;
};
// visualization main
struct Visualization{
  int headerId;
  String timestamp;
  String version;
  String manufacturer;
  String serialNumber;
  agvPositionObject agvPosition;

};

//************************************

//******<InstantActions>******* 
// Định nghĩa cấu trúc ActionParameter
struct ActionParameter {
  String key;

};

// Định nghĩa cấu trúc Action
struct Action {
  String actionType;
  String actionId;
  String actionDescription;
  String blockingType;
  ActionParameter actionParameters[10];
};

// Định nghĩa cấu trúc InstantActions
struct InstantActions {
  int headerId;
  String timestamp;
  String version;
  String manufacturer;
  String serialNumber;
  Action actions[10];
};

//************************************

#endif
