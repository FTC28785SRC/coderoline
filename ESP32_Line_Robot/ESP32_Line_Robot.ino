#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ============================================================
// ESP32 DevKit V4 + L298N + 2 DC motors + 5 digital IR sensors
// Arduino-ESP32 core 3.x (LEDC API: ledcAttach / ledcWrite)
// ============================================================

// Wi-Fi AP: connect phone/PC to this network, then open 192.168.4.1
const char* AP_SSID = "ESP32-LINE-ROBOT";
const char* AP_PASSWORD = "12345678";

// ----- 5 IR sensors (left to right) -----
constexpr uint8_t S1 = 26;
constexpr uint8_t S2 = 25;
constexpr uint8_t S3 = 32;
constexpr uint8_t S4 = 33;
constexpr uint8_t S5 = 27;

// Most digital IR modules output LOW on black.
// Change to HIGH if your modules output HIGH on black.
constexpr uint8_t BLACK_LEVEL = LOW;

// ----- L298N wiring -----
// Remove ENA and ENB jumpers on the L298N to use PWM.
constexpr uint8_t ENA = 21;
constexpr uint8_t IN1 = 22;
constexpr uint8_t IN2 = 23;
constexpr uint8_t IN3 = 18;
constexpr uint8_t IN4 = 19;
constexpr uint8_t ENB = 4;

constexpr uint32_t PWM_FREQ = 1000;
constexpr uint8_t PWM_BITS = 8;
constexpr int PWM_MAX = 255;

WebServer server(80);
Preferences prefs;

volatile bool robotRunning = false;
bool autoTuneRunning = false;
bool autoTuneRequested = false;
bool routeMode = false;
bool routeReady = false;

float Kp = 34.0f, Ki = 0.0f, Kd = 82.0f;
float integral = 0.0f;
float previousError = 0.0f;
float lastError = 0.0f;
int baseSpeed = 115;
int maxSpeed = 210;
int lastLinePosition = 0;
uint8_t sensorBits = 0;
int sensorCount = 0;
bool lineDetected = false;
unsigned long lastPidMicros = 0;
unsigned long lastStatusMs = 0;
unsigned long lostLineSince = 0;

constexpr int NODE_COUNT = 20;
int graph[NODE_COUNT][4] = {
  {1,5,-1,-1},{0,6,-1,-1},{3,7,-1,-1},{2,4,8,-1},{3,9,-1,-1},
  {0,6,10,-1},{1,5,7,11},{2,6,8,12},{3,7,9,13},{4,8,14,-1},
  {5,11,15,-1},{6,10,12,16},{7,11,13,17},{8,12,14,18},{9,13,19,-1},
  {10,16,-1,-1},{11,15,-1,-1},{12,18,-1,-1},{13,17,19,-1},{14,18,-1,-1}
};
int edgeWeights[NODE_COUNT][NODE_COUNT];
int route[NODE_COUNT + 1];
int routeLength = 0;
int routeIndex = 0;
int startNode = 0, endNode = 19;
int heading = 1; // 0=UP, 1=RIGHT, 2=DOWN, 3=LEFT; robot starts facing RIGHT
unsigned long lastNodeMs = 0;
bool intersectionLatched = false;

void handleRoot();
void handleStart();
void handleStop();
void handlePID();
void handleSave();
void handlePlan();
void handleRouteStart();
void handleAutoTune();
void handleStatus();
void runAutoTune();

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Line Robot</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#f1f5f9;color:#172033;font:14px Arial,sans-serif}
header{background:#172554;color:white;padding:16px 22px;display:flex;justify-content:space-between;gap:12px;align-items:center}
h1{font-size:20px;margin:0}main{max-width:1400px;margin:auto;padding:16px;display:grid;grid-template-columns:minmax(300px,1.5fr) minmax(290px,1fr);gap:16px}
.card{background:white;border-radius:12px;padding:16px;box-shadow:0 2px 10px #0f172a12;margin-bottom:14px}
h2{font-size:16px;margin:0 0 12px}.muted{color:#64748b;font-size:12px}
button,select,input{font:inherit}button{border:0;border-radius:8px;padding:10px 14px;background:#2563eb;color:white;font-weight:700;cursor:pointer}
button.secondary{background:#475569}button.danger{background:#dc2626}button.good{background:#059669}
button:disabled{opacity:.45}input[type=number]{width:82px;padding:7px;border:1px solid #cbd5e1;border-radius:7px}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}.row label{display:flex;align-items:center;gap:6px}
.sensor{display:inline-flex;flex-direction:column;align-items:center;gap:5px;padding:9px;border:1px solid #cbd5e1;border-radius:9px;min-width:48px}
.dot{width:18px;height:18px;border-radius:50%;background:#e2e8f0;border:1px solid #94a3b8}.dot.on{background:#111827}
.metric{background:#f8fafc;padding:10px;border-radius:8px;min-width:90px;flex:1}.metric b{display:block;font-size:19px;margin-top:4px}
.grid{display:grid;grid-template-columns:repeat(5,1fr);gap:8px}.node{border:2px solid #cbd5e1;background:white;color:#334155;border-radius:50%;width:40px;height:40px;padding:0}
.node.sel{background:#2563eb;color:white;border-color:#1d4ed8}.node.start{border-color:#dc2626}.node.end{border-color:#d97706}
.edgeRow{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #e2e8f0}
input[type=range]{width:100%}.pidrow{display:grid;grid-template-columns:42px 1fr 75px;gap:8px;align-items:center;margin:9px 0}
#status{font-weight:700}.ok{color:#059669}.bad{color:#dc2626}
@media(max-width:850px){main{grid-template-columns:1fr}header{align-items:flex-start;flex-direction:column}}
</style></head><body>
<header><div><h1>ESP32 Line Robot</h1><div class="muted" style="color:#bfdbfe">DevKit V4 · L298N · 5 IR sensors</div></div>
<div id="status">Đang kết nối...</div></header>
<main><section>
<div class="card"><h2>Giám sát cảm biến</h2><div class="row" id="sensors">
<div class="sensor">S1<div class="dot" id="d0"></div></div><div class="sensor">S2<div class="dot" id="d1"></div></div>
<div class="sensor">S3<div class="dot" id="d2"></div></div><div class="sensor">S4<div class="dot" id="d3"></div></div><div class="sensor">S5<div class="dot" id="d4"></div></div></div>
<div class="row" style="margin-top:12px"><div class="metric">Sai số<b id="err">0</b></div><div class="metric">Tốc độ trái/phải<b id="motors">0 / 0</b></div><div class="metric">Trạng thái line<b id="line">--</b></div></div></div>

<div class="card"><h2>Vẽ đường / lập tuyến</h2>
<p class="muted">Chọn nút bắt đầu và kết thúc trên lưới 5×4. Đây là sơ đồ tham khảo để bố trí đường line vật lý. Trọng số cạnh càng thấp thì tuyến càng ưu tiên.</p>
<div class="grid" id="nodes"></div>
<div class="row" style="margin-top:12px"><span>Nút đầu: <b id="start">0</b></span><span>Nút cuối: <b id="end">19</b></span>
<button onclick="findRoute()">Tìm đường tối ưu</button><button class="secondary" onclick="clearRoute()">Xóa chọn</button></div>
<p id="routeResult" class="muted">Chưa tính tuyến.</p>
<div id="weights" style="max-height:190px;overflow:auto"></div>
<p class="muted">Lưu ý: tuyến trên sơ đồ chỉ tương ứng với robot ngoài đời nếu đường kẻ có các giao điểm được bố trí tương ứng.</p>
</div>
<div class="card"><h2>Điều khiển robot</h2><div class="row">
<button class="good" onclick="post('/start')">START bám line</button><button class="danger" onclick="post('/stop')">STOP khẩn cấp</button>
<button class="secondary" onclick="post('/routeStart')">Chạy tuyến đã chọn</button></div>
<p class="muted">Luôn thử lần đầu khi bánh xe được kê khỏi mặt đất và giữ nút STOP sẵn sàng.</p></div>
</section>

<section>
<div class="card"><h2>Manual PID</h2><p class="muted">Thay đổi thông số trực tiếp trên ESP32. Nhấn Gửi để áp dụng.</p>
<div class="pidrow"><b>Kp</b><input id="kp" type="range" min="0" max="150" step=".5" value="34"><input id="kpv" type="number" step=".5" value="34"></div>
<div class="pidrow"><b>Ki</b><input id="ki" type="range" min="0" max="10" step=".01" value="0"><input id="kiv" type="number" step=".01" value="0"></div>
<div class="pidrow"><b>Kd</b><input id="kd" type="range" min="0" max="300" step=".5" value="82"><input id="kdv" type="number" step=".5" value="82"></div>
<div class="row"><label>Tốc độ cơ bản <input id="speed" type="number" min="50" max="220" value="115"></label><button onclick="sendPID()">Gửi PID</button><button class="secondary" onclick="post('/save')">Lưu vào ESP32</button></div>
</div>
<div class="card"><h2>Auto PID</h2><p class="muted">ESP32 lần lượt thử các bộ PID trong khi robot chạy trên đoạn line an toàn, sau đó chọn bộ có điểm đánh giá tốt nhất. Cần giám sát trực tiếp.</p>
<div class="row"><button class="good" onclick="post('/autotune')">Bắt đầu Auto PID</button><button class="danger" onclick="post('/stop')">Dừng thử</button></div>
<p id="tune">Chưa chạy Auto PID.</p><p class="muted">Bộ PID tốt nhất được nạp vào RAM; nhấn “Lưu vào ESP32” để lưu lâu dài.</p></div>
<div class="card"><h2>Thông số hiện tại</h2><div class="row"><div class="metric">Kp<b id="vkp">34</b></div><div class="metric">Ki<b id="vki">0</b></div><div class="metric">Kd<b id="vkd">82</b></div></div>
<p class="muted">Kết nối Wi-Fi: ESP32-LINE-ROBOT · Mật khẩu: 12345678 · Địa chỉ: 192.168.4.1</p></div>
</section></main>
<script>
const $=id=>document.getElementById(id);
let selectedStart=0,selectedEnd=19,weights={};
const edges=[];
for(let r=0;r<4;r++)for(let c=0;c<5;c++){let u=r*5+c;if(c<4)edges.push([u,u+1]);if(r<3)edges.push([u,u+5]);}
function buildMap(){
 $('nodes').innerHTML='';
 for(let i=0;i<20;i++){let b=document.createElement('button');b.className='node';b.textContent=i;b.onclick=()=>{if(selectedStart===null)selectedStart=i;else if(selectedEnd===null)selectedEnd=i;else {selectedStart=i;selectedEnd=null;} paintNodes();};$('nodes').appendChild(b);}
 $('weights').innerHTML=edges.map(([u,v],i)=>`<div class="edgeRow"><span>${u} ↔ ${v}</span><label>Trọng số <input type="number" min="1" max="99" value="${weights[i]||1}" onchange="weights[${i}]=Math.max(1,Number(this.value))"></label></div>`).join('');
 paintNodes();
}
function paintNodes(){[...$('nodes').children].forEach((b,i)=>{b.className='node'+(i===selectedStart?' start':'')+(i===selectedEnd?' end':'');});$('start').textContent=selectedStart??'--';$('end').textContent=selectedEnd??'--';}
function clearRoute(){selectedStart=null;selectedEnd=null;$('routeResult').textContent='Đã xóa lựa chọn.';paintNodes();}
function findRoute(){
 if(selectedStart===null||selectedEnd===null)return alert('Chọn nút đầu và nút cuối.');
 let d=Array(20).fill(Infinity),p=Array(20).fill(-1),used=Array(20).fill(false);d[selectedStart]=0;
 for(let k=0;k<20;k++){let u=-1;for(let i=0;i<20;i++)if(!used[i]&&(u<0||d[i]<d[u]))u=i;if(u<0||!isFinite(d[u]))break;used[u]=true;
 for(let [a,b] of edges){let v=a===u?b:(b===u?a:-1);if(v>=0){let idx=edges.findIndex(e=>(e[0]===a&&e[1]===b));let w=Number(weights[idx]||1);if(d[u]+w<d[v]){d[v]=d[u]+w;p[v]=u;}}}}
 let path=[];for(let v=selectedEnd;v!==-1;v=p[v])path.unshift(v);
 if(path[0]!==selectedStart)return $('routeResult').textContent='Không tìm thấy đường.';
 $('routeResult').textContent=`Tuyến tối ưu: ${path.join(' → ')} | Tổng trọng số: ${d[selectedEnd]}`;
 fetch('/plan',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'start='+selectedStart+'&end='+selectedEnd+'&weights='+encodeURIComponent(JSON.stringify(edges.map((e,i)=>({u:e[0],v:e[1],w:Number(weights[i]||1)}))))});
}
async function post(url){try{let r=await fetch(url,{method:'POST'});let t=await r.text();if(!r.ok)alert(t);}catch(e){alert('Không kết nối được ESP32');}}
function linkPair(a,b){$(a).oninput=()=>$(b).value=$(a).value;$(b).oninput=()=>$(a).value=$(b).value;}
linkPair('kp','kpv');linkPair('ki','kiv');linkPair('kd','kdv');
async function sendPID(){let q=new URLSearchParams({kp:$('kpv').value,ki:$('kiv').value,kd:$('kdv').value,speed:$('speed').value});let r=await fetch('/pid',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:q});if(!r.ok)alert(await r.text());}
async function refresh(){try{let s=await(await fetch('/status')).json();$('status').textContent='ESP32 kết nối';$('status').className='ok';
 for(let i=0;i<5;i++)$('d'+i).className='dot'+((s.bits&(1<<i))?' on':'');
 $('err').textContent=s.error;$('motors').textContent=s.left+' / '+s.right;$('line').textContent=s.line?'Đã thấy':'Mất line';
 $('vkp').textContent=s.kp;$('vki').textContent=s.ki;$('vkd').textContent=s.kd;
 $('kp').value=$('kpv').value=s.kp;$('ki').value=$('kiv').value=s.ki;$('kd').value=$('kdv').value=s.kd;$('speed').value=s.speed;
 $('tune').textContent=s.tuning?`Đang thử Auto PID · ứng viên ${s.candidate}/7 · tốt nhất: ${s.best}`:(s.tuneDone?'Hoàn tất Auto PID. Bộ tốt nhất: '+s.best:'Auto PID chưa chạy.');
 }catch(e){$('status').textContent='Mất kết nối';$('status').className='bad';}}
buildMap();setInterval(refresh,350);refresh();
</script></body></html>
)HTML";

// Motor output state for telemetry
int currentLeftPWM = 0, currentRightPWM = 0;

void setOneMotor(uint8_t en, uint8_t inA, uint8_t inB, int speed) {
  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  if (speed > 0) { digitalWrite(inA, HIGH); digitalWrite(inB, LOW); }
  else if (speed < 0) { digitalWrite(inA, LOW); digitalWrite(inB, HIGH); }
  else { digitalWrite(inA, LOW); digitalWrite(inB, LOW); }
  ledcWrite(en, abs(speed));
}

void setMotors(int left, int right) {
  currentLeftPWM = constrain(left, -PWM_MAX, PWM_MAX);
  currentRightPWM = constrain(right, -PWM_MAX, PWM_MAX);
  // Left motor uses IN3/IN4 + ENB; right motor uses IN1/IN2 + ENA.
  setOneMotor(ENB, IN3, IN4, currentLeftPWM);
  setOneMotor(ENA, IN1, IN2, currentRightPWM);
}

void stopMotors() {
  setMotors(0, 0);
}

void readSensors() {
  const uint8_t pins[5] = {S1,S2,S3,S4,S5};
  const int weights[5] = {-2000,-1000,0,1000,2000};
  long sum = 0;
  sensorBits = 0;
  sensorCount = 0;
  for (int i=0;i<5;i++) {
    bool black = (digitalRead(pins[i]) == BLACK_LEVEL);
    if (black) {
      sensorBits |= (1 << i);
      sum += weights[i];
      sensorCount++;
    }
  }
  lineDetected = sensorCount > 0;
  if (lineDetected) {
    lastLinePosition = (int)(sum / sensorCount);
  }
}

void followLinePID() {
  readSensors();
  if (!lineDetected) {
    if (lostLineSince == 0) lostLineSince = millis();
    int search = (lastError >= 0) ? 95 : -95;
    setMotors(search, -search);
    return;
  }
  lostLineSince = 0;
  float error = (float)lastLinePosition / 1000.0f;
  unsigned long now = micros();
  float dt = (lastPidMicros == 0) ? 0.01f : (now-lastPidMicros)/1000000.0f;
  lastPidMicros = now;
  dt = constrain(dt, 0.001f, 0.05f);
  integral += error * dt;
  integral = constrain(integral, -20.0f, 20.0f);
  float derivative = (error - previousError) / dt;
  float correction = Kp*error + Ki*integral + Kd*derivative;
  previousError = error;
  lastError = error;
  int left = constrain((int)(baseSpeed + correction), -maxSpeed, maxSpeed);
  int right = constrain((int)(baseSpeed - correction), -maxSpeed, maxSpeed);
  setMotors(left,right);
}

bool isIntersection() {
  readSensors();
  // This threshold is only a starting point; tune for the width of your track.
  return sensorCount >= 4;
}

int directionOf(int from, int to) {
  if (to == from - 5) return 0; // UP
  if (to == from + 1) return 1; // RIGHT
  if (to == from + 5) return 2; // DOWN
  if (to == from - 1) return 3; // LEFT
  return -1;
}

void turnRelative(int delta) {
  // delta: 1 = right, -1 = left, 2 = U-turn
  if (delta == 0) return;
  int turnTime = (delta == 2) ? 650 : 360;
  if (delta == 1) setMotors(125,-125);
  else if (delta == -1) setMotors(-125,125);
  else setMotors(125,-125);
  delay(180); // clear current intersection
  unsigned long started = millis();
  while (millis()-started < (unsigned long)turnTime) {
    server.handleClient();
    readSensors();
    delay(2);
  }
  started = millis();
  while (millis()-started < 900) {
    readSensors();
    if (sensorBits & (1<<2)) break;
    server.handleClient();
    delay(2);
  }
  stopMotors();
  delay(60);
  integral = 0;
  previousError = 0;
}

void driveRouteStep(int from, int to) {
  int newHeading = directionOf(from,to);
  if (newHeading < 0) return;
  int delta = (newHeading - heading + 4) % 4;
  if (delta == 1) turnRelative(1);
  else if (delta == 3) turnRelative(-1);
  else if (delta == 2) turnRelative(2);
  else {
    // Move clear of the wide intersection before detecting the next one.
    setMotors(baseSpeed, baseSpeed);
    unsigned long t0 = millis();
    while (millis() - t0 < 180) { server.handleClient(); delay(2); }
    stopMotors();
  }
  heading = newHeading;
}

void calculateDijkstra(int start, int goal) {
  int dist[NODE_COUNT], parent[NODE_COUNT];
  bool used[NODE_COUNT] = {};
  for (int i=0;i<NODE_COUNT;i++) { dist[i]=100000; parent[i]=-1; }
  dist[start]=0;
  for (int k=0;k<NODE_COUNT;k++) {
    int u=-1;
    for (int i=0;i<NODE_COUNT;i++) if(!used[i] && (u<0 || dist[i]<dist[u])) u=i;
    if(u<0 || dist[u]>=100000) break;
    used[u]=true;
    if(u==goal) break;
    for(int j=0;j<4;j++) {
      int v=graph[u][j];
      if(v>=0 && !used[v] && dist[u]+edgeWeights[u][v]<dist[v]) {
        dist[v]=dist[u]+edgeWeights[u][v]; parent[v]=u;
      }
    }
  }
  routeLength=0;
  if(start!=goal && parent[goal]<0) return;
  int reversePath[NODE_COUNT], n=0;
  for(int v=goal;v!=-1 && n<NODE_COUNT;v=parent[v]) reversePath[n++]=v;
  if(n==0 || reversePath[n-1]!=start) return;
  for(int i=n-1;i>=0;i--) route[routeLength++]=reversePath[i];
  routeReady=true;
}

void handleRoot() { server.send_P(200,"text/html",INDEX_HTML); }

void handleStart() {
  routeMode=false; robotRunning=true; autoTuneRunning=false;
  integral=0; previousError=0; lastPidMicros=0; lostLineSince=0;
  server.send(200,"text/plain","Line following started");
}

void handleStop() {
  robotRunning=false; routeMode=false; autoTuneRunning=false; autoTuneRequested=false;
  stopMotors();
  server.send(200,"text/plain","Robot stopped");
}

void handlePID() {
  if(server.hasArg("kp")) Kp=constrain(server.arg("kp").toFloat(),0.0f,150.0f);
  if(server.hasArg("ki")) Ki=constrain(server.arg("ki").toFloat(),0.0f,10.0f);
  if(server.hasArg("kd")) Kd=constrain(server.arg("kd").toFloat(),0.0f,300.0f);
  if(server.hasArg("speed")) baseSpeed=constrain(server.arg("speed").toInt(),50,220);
  integral=0; previousError=0;
  server.send(200,"text/plain","PID updated");
}

void handleSave() {
  prefs.begin("linebot",false);
  prefs.putFloat("kp",Kp); prefs.putFloat("ki",Ki); prefs.putFloat("kd",Kd);
  prefs.putInt("speed",baseSpeed);
  prefs.end();
  server.send(200,"text/plain","PID saved to flash");
}

void handlePlan() {
  if(!server.hasArg("start") || !server.hasArg("end")) {
    server.send(400,"text/plain","Missing start/end"); return;
  }
  int s=server.arg("start").toInt(), e=server.arg("end").toInt();
  if(s<0||s>=NODE_COUNT||e<0||e>=NODE_COUNT) {
    server.send(400,"text/plain","Invalid node"); return;
  }
  // Web client sends edge weights as JSON. Parse compactly without ArduinoJson.
  String data=server.arg("weights");
  for(int i=0;i<NODE_COUNT;i++)for(int j=0;j<NODE_COUNT;j++)edgeWeights[i][j]=1000;
  for(int u=0;u<NODE_COUNT;u++)for(int k=0;k<4;k++){int v=graph[u][k];if(v>=0)edgeWeights[u][v]=1;}
  int pos=0;
  while((pos=data.indexOf("\"u\"",pos))>=0) {
    int colon=data.indexOf(':',pos); int comma=data.indexOf(',',colon);
    int u=data.substring(colon+1,comma).toInt();
    pos=data.indexOf("\"v\"",comma); if(pos<0)break;
    colon=data.indexOf(':',pos); comma=data.indexOf(',',colon);
    int v=data.substring(colon+1,comma).toInt();
    pos=data.indexOf("\"w\"",comma); if(pos<0)break;
    colon=data.indexOf(':',pos); int end=data.indexOf('}',colon);
    int w=data.substring(colon+1,end).toInt();
    if(u>=0&&u<NODE_COUNT&&v>=0&&v<NODE_COUNT&&w>=1&&w<=99){edgeWeights[u][v]=w;edgeWeights[v][u]=w;}
    pos=end+1;
  }
  startNode=s; endNode=e; routeReady=false;
  calculateDijkstra(s,e);
  if(!routeReady){server.send(404,"text/plain","No route");return;}
  String out="{\"path\":[";
  for(int i=0;i<routeLength;i++){if(i)out+=",";out+=String(route[i]);}
  out+="]}";
  server.send(200,"application/json",out);
}

void handleRouteStart() {
  if(!routeReady || routeLength<2){server.send(400,"text/plain","Calculate a route first");return;}
  routeMode=true; robotRunning=true; routeIndex=0; lastNodeMs=millis();
  // Default initial heading is RIGHT; orient the robot this way before starting.
  integral=0; previousError=0;
  server.send(200,"text/plain","Route navigation started");
}

void handleAutoTune() {
  if(robotRunning){server.send(409,"text/plain","Stop the robot before starting Auto PID");return;}
  autoTuneRequested=true; autoTuneRunning=false;
  server.send(200,"text/plain","Auto PID queued. Place robot on a safe test line.");
}

int tuneCandidate=0;
int bestTune=0;
const char* bestTuneName="none";
float bestTuneScore=1e30f;
float tuneScore=0;
unsigned long tuneStarted=0;
bool tuneDone=false;

void handleStatus() {
  readSensors();
  String json="{";
  json+="\"bits\":"+String(sensorBits)+",\"error\":"+String(lastLinePosition);
  json+=",\"left\":"+String(currentLeftPWM)+",\"right\":"+String(currentRightPWM);
  json+=",\"line\":"+String(lineDetected?1:0);
  json+=",\"kp\":"+String(Kp,2)+",\"ki\":"+String(Ki,3)+",\"kd\":"+String(Kd,2);
  json+=",\"speed\":"+String(baseSpeed);
  json+=",\"tuning\":"+String(autoTuneRunning?1:0);
  json+=",\"candidate\":"+String(tuneCandidate+1)+",\"best\":\""+String(bestTuneName)+"\"";
  json+=",\"tuneDone\":"+String(tuneDone?1:0)+"}";
  server.send(200,"application/json",json);
}

// Auto PID: supervised candidate testing on a straight/oval test track.
// Score = accumulated absolute line error + penalty for losing the line.
struct TuneSet { float kp,ki,kd; const char* name; };
TuneSet tuneSets[] = {
  {22,0,45,"22/0/45"}, {28,0,60,"28/0/60"}, {34,0,82,"34/0/82"},
  {40,0,95,"40/0/95"}, {46,0,110,"46/0/110"}, {32,0.15f,75,"32/0.15/75"},
  {38,0.10f,90,"38/0.10/90"}
};
constexpr int TUNE_COUNT=7;
void startTuneCandidate() {
  if(tuneCandidate>=TUNE_COUNT) {
    autoTuneRunning=false; tuneDone=true;
    Kp=tuneSets[bestTune].kp; Ki=tuneSets[bestTune].ki; Kd=tuneSets[bestTune].kd;
    robotRunning=false; stopMotors();
    return;
  }
  Kp=tuneSets[tuneCandidate].kp; Ki=tuneSets[tuneCandidate].ki; Kd=tuneSets[tuneCandidate].kd;
  tuneScore=0; tuneStarted=millis(); integral=0; previousError=0; lastPidMicros=0;
}

void runAutoTune() {
  if(autoTuneRequested) {
    autoTuneRequested=false; autoTuneRunning=true; robotRunning=true;
    tuneCandidate=0; bestTune=0; bestTuneScore=1e30f; tuneDone=false;
    startTuneCandidate();
  }
  if(!autoTuneRunning) return;
  readSensors();
  followLinePID();
  if(lineDetected) tuneScore += abs(lastLinePosition);
  else tuneScore += 7000;
  if(millis()-tuneStarted>=6000) {
    if(tuneScore<bestTuneScore) {
      bestTuneScore=tuneScore; bestTune=tuneCandidate;
      bestTuneName=tuneSets[bestTune].name;
    }
    tuneCandidate++;
    startTuneCandidate();
  }
}

void setup() {
  Serial.begin(115200);
  const uint8_t sensorPins[5]={S1,S2,S3,S4,S5};
  for(uint8_t p:sensorPins) pinMode(p,INPUT);
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
  ledcAttach(ENA,PWM_FREQ,PWM_BITS);
  ledcAttach(ENB,PWM_FREQ,PWM_BITS);
  stopMotors();

  prefs.begin("linebot",true);
  Kp=prefs.getFloat("kp",Kp); Ki=prefs.getFloat("ki",Ki); Kd=prefs.getFloat("kd",Kd);
  baseSpeed=prefs.getInt("speed",baseSpeed);
  prefs.end();

  for(int i=0;i<NODE_COUNT;i++)for(int j=0;j<NODE_COUNT;j++)edgeWeights[i][j]=1000;
  for(int u=0;u<NODE_COUNT;u++)for(int k=0;k<4;k++){int v=graph[u][k];if(v>=0)edgeWeights[u][v]=1;}

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID,AP_PASSWORD);
  Serial.print("Connect to Wi-Fi: "); Serial.println(AP_SSID);
  Serial.print("Open browser: http://"); Serial.println(WiFi.softAPIP());

  server.on("/",HTTP_GET,handleRoot);
  server.on("/start",HTTP_POST,handleStart);
  server.on("/stop",HTTP_POST,handleStop);
  server.on("/pid",HTTP_POST,handlePID);
  server.on("/save",HTTP_POST,handleSave);
  server.on("/plan",HTTP_POST,handlePlan);
  server.on("/routeStart",HTTP_POST,handleRouteStart);
  server.on("/autotune",HTTP_POST,handleAutoTune);
  server.on("/status",HTTP_GET,handleStatus);
  server.begin();
}

void loop() {
  server.handleClient();

  if(autoTuneRequested || autoTuneRunning) {
    runAutoTune();
    delay(5);
    return;
  }

  if(robotRunning) {
    if(routeMode && routeLength>1) {
      followLinePID();
      if(isIntersection() && millis()-lastNodeMs>1000) {
        intersectionLatched=true;
        int from=route[routeIndex], to=route[routeIndex+1];
        stopMotors();
        driveRouteStep(from,to);
        routeIndex++;
        lastNodeMs=millis();
        if(routeIndex>=routeLength-1) {
          robotRunning=false; routeMode=false; stopMotors();
        }
      }
      if(!isIntersection()) intersectionLatched=false;
    } else {
      followLinePID();
    }
  } else {
    stopMotors();
  }
  delay(5);
}
