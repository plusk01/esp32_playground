data = csvread('/tmp/data.csv');

% data = data(1:500,:);

seq = data(:,1);
time = data(:,2);
acc = data(:,3:5);
gyr = data(:,6:8);

figure(1), clf;
plot(seq, acc);
xlabel('Sequence');
ylabel('Accelerometer');

figure(2), clf;
plot(seq, gyr);
xlabel('Sequence');
ylabel('Gyro');

dt = mean(diff(time))

figure(3), clf;
stem(diff(time));
hold on
plot(seq, mean(diff(time))*ones(size(seq)))
plot(seq, 1/1660*ones(size(seq)))