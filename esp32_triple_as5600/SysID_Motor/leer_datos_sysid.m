%% leer_datos_sysid.m
% Lee los datos de la prueba PRBS enviados por el ESP32 (SysID_Motor.ino)
% por el puerto serial, y arma objetos iddata listos para el System
% Identification Toolbox.
%
% Antes de correr:
%   1. Sube SysID_Motor.ino al ESP32 y cablea el motor/driver/encoder bajo
%      prueba como se indica en los comentarios de ese sketch.
%   2. Cierra el Monitor Serial del IDE de Arduino (el puerto serial solo
%      lo puede usar un programa a la vez).
%   3. Ajusta PUERTO abajo a tu puerto real:
%        Windows: revisa el Administrador de dispositivos (ej. "COM3")
%        Mac:     ls /dev/cu.usbserial-*  o  /dev/cu.SLAB_USBtoUART
%        Linux:   ls /dev/ttyUSB*

PUERTO = "COM3";      % <-- AJUSTA ESTO a tu puerto real
BAUD   = 115200;
Ts     = 0.010;       % 10 ms - debe coincidir con TS_MS del sketch

s = serialport(PUERTO, BAUD);
configureTerminator(s, "LF");
flush(s);

pause(1.5); % da tiempo a que el ESP32 termine de imprimir el mensaje de arranque
flush(s);

fprintf("Enviando START...\n");
writeline(s, "START");

t_ms   = [];
duty   = [];
posRaw = [];

leyendoDatos = false;
while true
    linea = readline(s);
    linea = strtrim(linea);

    if linea == "t_ms,duty,pos_raw"
        leyendoDatos = true;
        continue;
    end
    if linea == "FIN" || linea == "ABORTADO"
        fprintf("Prueba terminada: %s\n", linea);
        break;
    end
    if leyendoDatos
        partes = split(linea, ",");
        if numel(partes) == 3
            t_ms(end+1)   = str2double(partes(1)); %#ok<AGROW>
            duty(end+1)   = str2double(partes(2)); %#ok<AGROW>
            posRaw(end+1) = str2double(partes(3)); %#ok<AGROW>
        end
    end
end

clear s

if isempty(t_ms)
    error("No se recibieron datos. Revisa el puerto/baudrate y que el sketch este cargado y respondiendo.");
end

fprintf("Muestras recibidas: %d (~%.1f s)\n", numel(t_ms), (t_ms(end)-t_ms(1))/1000);

%% Conversion de unidades
t      = (t_ms - t_ms(1)) / 1000;   % segundos, empieza en 0
u      = duty(:);                    % entrada: duty PWM aplicado (0-255)
posDeg = posRaw(:) * (360/4096);     % posicion continua en grados (sin wraparound)
velDeg = gradient(posDeg, t);        % velocidad estimada por diferenciacion (deg/s)

%% Construir objetos iddata (uno para posicion, otro para velocidad)
data_pos = iddata(posDeg, u, Ts, 'Name', 'Motor - posicion');
data_pos.InputName  = 'Duty PWM (0-255)';
data_pos.OutputName = 'Posicion (deg)';
data_pos.TimeUnit   = 'seconds';

data_vel = iddata(velDeg, u, Ts, 'Name', 'Motor - velocidad');
data_vel.InputName  = 'Duty PWM (0-255)';
data_vel.OutputName = 'Velocidad (deg/s)';
data_vel.TimeUnit   = 'seconds';

%% Graficas rapidas para inspeccion visual
figure('Name', 'Datos crudos SysID');
subplot(3,1,1);
plot(t, u); ylabel('Duty (0-255)'); grid on;
title('Senal PRBS aplicada');
subplot(3,1,2);
plot(t, posDeg); ylabel('Posicion (deg)'); grid on;
subplot(3,1,3);
plot(t, velDeg); ylabel('Velocidad (deg/s)'); xlabel('Tiempo (s)'); grid on;

%% Guardar
nombreArchivo = sprintf('sysid_motor_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(nombreArchivo, 't', 'u', 'posDeg', 'velDeg', 'data_pos', 'data_vel');
fprintf("Datos guardados en %s\n", nombreArchivo);

%% Siguiente paso: System Identification Toolbox
fprintf("\nPara identificar la planta:\n");
fprintf("  - App grafica: ejecuta  systemIdentification  (o el alias 'ident'),\n");
fprintf("    despues Import data > Data object, y selecciona data_pos o data_vel.\n");
fprintf("  - Linea de comandos, estimacion rapida de 1er orden (planta de velocidad tipica de un motor DC):\n");
fprintf("      sys = tfest(data_vel, 1);\n");
fprintf("      compare(data_vel, sys);\n");
