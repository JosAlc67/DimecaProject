%% leer_datos_sysid.m
% Lee los datos de la prueba PRBS enviados por el ESP32 (SysID_Motor.ino)
% por el puerto serial, y arma objetos iddata listos para el System
% Identification Toolbox. Ya NO hace falta recablear nada entre motores:
% el sketch prueba cualquiera de los 3 por comando (START1/START2/START3).
%
% Antes de correr:
%   1. Sube SysID_Motor.ino al ESP32 (los 3 motores ya deben estar
%      permanentemente cableados como en el proyecto principal).
%   2. Cierra el Monitor Serial del IDE de Arduino (el puerto serial solo
%      lo puede usar un programa a la vez).
%   3. Ajusta PUERTO abajo a tu puerto real, y MOTOR al que quieras probar.

PUERTO = "COM3";      % <-- AJUSTA ESTO a tu puerto real
BAUD   = 115200;
Ts     = 0.010;       % 10 ms - debe coincidir con TS_MS del sketch
MOTOR  = 1;            % <-- 1, 2 o 3: cual motor probar en esta corrida

comando = sprintf("START%d", MOTOR);

s = serialport(PUERTO, BAUD);
configureTerminator(s, "LF");
flush(s);

pause(1.5); % da tiempo a que el ESP32 termine de imprimir el mensaje de arranque
flush(s);

fprintf("Enviando %s...\n", comando);
writeline(s, comando);

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

% Quita marcas de tiempo duplicadas/no crecientes antes de derivar (si dos
% muestras quedan con el mismo t, gradient() divide entre cero y mete NaN
% que se propaga a los vecinos).
[t_unico, idxUnico] = unique(t, 'stable');
posDeg_unico = posDeg(idxUnico);
velDeg = nan(size(posDeg));
velDeg(idxUnico) = gradient(posDeg_unico, t_unico);
if any(isnan(velDeg))
    fprintf("Aviso: %d muestra(s) con timestamp duplicado, se interpolan para la velocidad.\n", sum(isnan(velDeg)));
    velDeg = fillmissing(velDeg, 'linear');
end

%% Construir objetos iddata (uno para posicion, otro para velocidad)
data_pos = iddata(posDeg, u, Ts, 'Name', sprintf('Motor %d - posicion', MOTOR));
data_pos.InputName  = 'Duty PWM (0-255)';
data_pos.OutputName = 'Posicion (deg)';
data_pos.TimeUnit   = 'seconds';

data_vel = iddata(velDeg, u, Ts, 'Name', sprintf('Motor %d - velocidad', MOTOR));
data_vel.InputName  = 'Duty PWM (0-255)';
data_vel.OutputName = 'Velocidad (deg/s)';
data_vel.TimeUnit   = 'seconds';

%% Graficas rapidas para inspeccion visual
figure('Name', sprintf('Datos crudos SysID - Motor %d', MOTOR));
subplot(3,1,1);
plot(t, u); ylabel('Duty (0-255)'); grid on;
title(sprintf('Senal PRBS aplicada - Motor %d', MOTOR));
subplot(3,1,2);
plot(t, posDeg); ylabel('Posicion (deg)'); grid on;
subplot(3,1,3);
plot(t, velDeg); ylabel('Velocidad (deg/s)'); xlabel('Tiempo (s)'); grid on;

%% Guardar
nombreArchivo = sprintf('sysid_motor%d_%s.mat', MOTOR, datestr(now, 'yyyymmdd_HHMMSS'));
save(nombreArchivo, 't', 'u', 'posDeg', 'velDeg', 'data_pos', 'data_vel', 'MOTOR');
fprintf("Datos guardados en %s\n", nombreArchivo);

%% Siguiente paso: System Identification Toolbox
fprintf("\nPara identificar la planta:\n");
fprintf("  - App grafica: ejecuta  systemIdentification  (o el alias 'ident'),\n");
fprintf("    despues Import data > Data object, y selecciona data_pos o data_vel.\n");
fprintf("  - Linea de comandos, estimacion rapida de 1er orden (planta de velocidad tipica de un motor DC):\n");
fprintf("      sys = tfest(data_vel, 1);\n");
fprintf("      compare(data_vel, sys);\n");
