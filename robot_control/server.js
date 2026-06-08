const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const dgram = require('dgram');
const path = require('path');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

const UDP_PORT = 7856;
const UDP_IP = '239.1.1.1';
const HTTP_PORT = 8080;

const udpClient = dgram.createSocket('udp4');

// Раздаём статические файлы
app.use(express.static(__dirname));

// Главная страница
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});

// WebSocket соединение
wss.on('connection', (ws) => {
    console.log('🔌 WebSocket client connected');
    
    ws.on('message', (message) => {
        try {
            const data = JSON.parse(message);
            
            if (data.type === 'motion') {
                let speed = parseFloat(data.speed) || 0;
                let angle = parseFloat(data.angle) || 0;
                let omega = parseFloat(data.omega) || 0;
                
                const MAX_SPEED = 1.0;
                const MAX_OMEGA = 2.0;
                
                speed = Math.min(Math.max(speed, 0), MAX_SPEED);
                angle = Math.min(Math.max(angle, -Math.PI), Math.PI);
                omega = Math.min(Math.max(omega, -MAX_OMEGA), MAX_OMEGA);
                
                const packet = `command;3;${speed.toFixed(3)};${angle.toFixed(3)};${omega.toFixed(3)};`;
                console.log(packet);
                
                udpClient.send(packet, UDP_PORT, UDP_IP, (err) => {
                    if (err) console.error('UDP error:', err);
                });
                
                console.log(`📤 ${packet}`);
            }
            
            else if (data.type === 'stop') {
                udpClient.send('command;3;0;0;0;', UDP_PORT, UDP_IP);
                console.log('⏹ Stop command sent');
            }
            
        } catch (err) {
            console.error('Parse error:', err);
        }
    });
    
    ws.on('close', () => {
        console.log('🔌 WebSocket client disconnected');
    });
});

server.listen(HTTP_PORT, '0.0.0.0', () => {
    console.log(`🌐 HTTP + WebSocket server: http://localhost:${HTTP_PORT}`);
    console.log(`📡 UDP multicast: ${UDP_IP}:${UDP_PORT}`);
    console.log(`📱 Доступ с телефона: http://<IP_компьютера>:${HTTP_PORT}`);
});