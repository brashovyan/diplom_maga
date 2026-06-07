const express = require('express');
const dgram = require('dgram');
const path = require('path');

const app = express();
const UDP_PORT = 7856;
const UDP_IP = '239.1.1.1';
const HTTP_PORT = 8080;

const udpClient = dgram.createSocket('udp4');

// Раздаём статические файлы из текущей папки
app.use(express.static(__dirname));

// API для движения
app.get('/motion', (req, res) => {
    // Параметры уже готовы: x = speed, y = angle, rot = omega
    let speed = parseFloat(req.query.x) || 0;
    let angle = parseFloat(req.query.y) || 0;
    let omega = parseFloat(req.query.rot) || 0;
    
    // Ограничения
    const MAX_SPEED = 1.0;
    const MAX_OMEGA = 2.0;
    
    speed = Math.min(Math.max(speed, 0), MAX_SPEED);
    angle = Math.min(Math.max(angle, -Math.PI), Math.PI);
    omega = Math.min(Math.max(omega, -MAX_OMEGA), MAX_OMEGA);
    
    const packet = `command;3;${speed.toFixed(3)};${angle.toFixed(3)};${omega.toFixed(3)};`;
    
    udpClient.send(packet, UDP_PORT, UDP_IP, (err) => {
        if (err) console.error('UDP error:', err);
    });
    
    console.log(`${packet}`);
    res.send('OK');
});

// API для стоп
app.get('/stop', (req, res) => {
    udpClient.send('command;3;0;0;0;', UDP_PORT, UDP_IP);
    res.send('OK');
    console.log('stop');
});

// Главная страница - отдаёт ваш HTML
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});

app.listen(HTTP_PORT, '0.0.0.0', () => {
    console.log(`🌐 Server: http://localhost:${HTTP_PORT}`);
    console.log(`📡 UDP multicast: ${UDP_IP}:${UDP_PORT}`);
    console.log(`📱 Доступ с телефона: http://<IP_компьютера>:${HTTP_PORT}`);
});