%% Đọc dữ liệu đầu vào từ file csv
data = readmatrix('ketqua175.csv');
data = rmmissing(data);

%% Tạo vector đầu vào và đầu ra
% Vector u: Đầu vào (Điện áp)
u = data(:, 2); 

% Vector y: Đầu ra (Góc quay)
y = data(:, 3); 
y = y - y(1); % Ép góc xuất phát về 0

% Thời gian lấy mẫu
Ts = 0.005; 

% Gói u và y lại thành my_data để dưa vào Toolbox
new_data1 = iddata(y, u, Ts, 'InputName', 'Voltage', 'OutputName', 'Angle');

%% Mở System Identification Toolbox
systemIdentification()