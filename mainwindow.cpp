#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow),
    m_serial(new QSerialPort(this)),
    m_status(new QLabel)
{
    m_ui->setupUi(this);
    m_ui->statusBar->addWidget(m_status);

    serial_init();

    camera_init();

    timer_init();
}

MainWindow::~MainWindow()
{
    delete m_ui;
    delete m_serial;
    delete m_status;
    m_camera.reset();
    delete timer_camera;
    delete timer_serial;
}

void MainWindow::camera_init() {
    camera_updateDevice();
    connect(m_ui->comboBox_CameraDevice, &QComboBox::currentTextChanged, this, &MainWindow::camera_setDevice);

}

void MainWindow::camera_updateDevice() {
    const QList<QCameraInfo> availableCameras = QCameraInfo::availableCameras();
    int count_port = availableCameras.count();
    int count_item = m_ui->comboBox_CameraDevice->count();

    if(count_port < count_item) {
        for(int i = count_port; i < count_item; i++) {
            m_ui->comboBox_CameraDevice->removeItem(i);
        }
        count_item = count_port;
    } else if(count_port > count_item) {
        for(int i = count_item; i < count_port; i++) {
            m_ui->comboBox_CameraDevice->addItem(availableCameras.at(i).description());
        }
    }
    for(int i = 0 ; i < count_item; i ++) {
        if(availableCameras.at(i).description() != m_ui->comboBox_CameraDevice->itemText(i)) {
            m_ui->comboBox_CameraDevice->removeItem(i);
            m_ui->comboBox_CameraDevice->addItem(availableCameras.at(i).description());
        }
    }
}

void MainWindow::camera_setDevice() {
    const QList<QCameraInfo> availableCameras = QCameraInfo::availableCameras();
    for (const QCameraInfo &cameraInfo : availableCameras) {
        if(cameraInfo.description() == m_ui->comboBox_CameraDevice->currentText()) {
            m_camera.reset(new QCamera(cameraInfo));
            m_img.reset(new QAbstractVideoSurface(this));
            m_camera->setViewfinder(m_img);
            timer_frame(20);
        }
    }
    connect(m_camera.data(), QOverload<QCamera::Error>::of(&QCamera::error), this, &MainWindow::camera_handleError );
    connect(m_camera.data(), &QCamera::stateChanged, this, &MainWindow::camera_stateControll );
    connect(timer_frame, &QTimer::timeout, this, &MainWindow::image_processing);
}

void MainWindow::camera_stateControll(QCamera::State state)
{
    switch (state) {
    case QCamera::ActiveState:
        m_ui->pushButton_Camera_Connect->setText(tr("Disconnect"));
        m_ui->comboBox_CameraDevice->setEnabled(false);
        timer_camera->stop();
        break;
    case QCamera::UnloadedState:
    case QCamera::LoadedState:
        m_ui->pushButton_Camera_Connect->setText(tr("Connect"));
        m_ui->comboBox_CameraDevice->setEnabled(true);
        timer_camera->start(1000);
    }
}

void MainWindow::camera_openCamera() {
        m_camera->start();
}

void MainWindow::camera_closeCamera() {
        m_camera->stop();
}

void MainWindow::camera_handleError() {
    QMessageBox::critical(this, tr("Critical Error"), m_camera->errorString());
//    camera_closeCamera();
}

void MainWindow::timer_init() {
    timer_serial = new QTimer(this);
    connect(timer_serial, &QTimer::timeout, this, &MainWindow::serial_updatePortName);
    timer_serial->start(1000);

    timer_camera = new QTimer(this);
    connect(timer_camera, &QTimer::timeout, this, &MainWindow::camera_updateDevice);
    timer_camera->start(1000);
}

void MainWindow::serial_init() {
    m_ui->comboBox_Baudrate->addItem(QStringLiteral("9600"), QSerialPort::Baud9600);
    m_ui->comboBox_Baudrate->addItem(QStringLiteral("19200"), QSerialPort::Baud19200);
    m_ui->comboBox_Baudrate->addItem(QStringLiteral("38400"), QSerialPort::Baud38400);
    m_ui->comboBox_Baudrate->addItem(QStringLiteral("115200"), QSerialPort::Baud115200);

    m_ui->comboBox_Databits->addItem(QStringLiteral("5"), QSerialPort::Data5);
    m_ui->comboBox_Databits->addItem(QStringLiteral("6"), QSerialPort::Data6);
    m_ui->comboBox_Databits->addItem(QStringLiteral("7"), QSerialPort::Data7);
    m_ui->comboBox_Databits->addItem(QStringLiteral("8"), QSerialPort::Data8);

    m_ui->comboBox_Parity->addItem(tr("None"), QSerialPort::NoParity);
    m_ui->comboBox_Parity->addItem(tr("Even"), QSerialPort::EvenParity);
    m_ui->comboBox_Parity->addItem(tr("Odd"), QSerialPort::OddParity);
    m_ui->comboBox_Parity->addItem(tr("Mark"), QSerialPort::MarkParity);
    m_ui->comboBox_Parity->addItem(tr("Space"), QSerialPort::SpaceParity);

    m_ui->comboBox_Stopbits->addItem(QStringLiteral("1"), QSerialPort::OneStop);
#ifdef Q_OS_WIN
    m_ui->comboBox_Stopbits->addItem(tr("1.5"), QSerialPort::OneAndHalfStop);
#endif
    m_ui->comboBox_Stopbits->addItem(QStringLiteral("2"), QSerialPort::TwoStop);

    m_ui->comboBox_Flowcontrol->addItem(tr("None"), QSerialPort::NoFlowControl);
    m_ui->comboBox_Flowcontrol->addItem(tr("RTS/CTS"), QSerialPort::HardwareControl);
    m_ui->comboBox_Flowcontrol->addItem(tr("XON/XOFF"), QSerialPort::SoftwareControl);

    serial_updatePortName();
    serial_setDefault();
    serial_updateSetting();
    statusBar_Message(tr("Disconnected"));

    connect(m_ui->comboBox_Comport, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );
    connect(m_ui->comboBox_Baudrate, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );
    connect(m_ui->comboBox_Databits, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );
    connect(m_ui->comboBox_Parity, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );
    connect(m_ui->comboBox_Stopbits, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );
    connect(m_ui->comboBox_Flowcontrol, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );

    connect(m_serial, &QSerialPort::errorOccurred, this, &MainWindow::serial_handleError);
    connect(m_serial, &QSerialPort::readyRead, this, &MainWindow::serial_read);
    connect(m_ui->pushButton_LogsClear, &QPushButton::clicked, this, &MainWindow::logs_clear);

}

void MainWindow::serial_setDefault() {
    m_ui->comboBox_Baudrate->setCurrentText("115200");
    m_ui->comboBox_Databits->setCurrentText("8");
    m_ui->comboBox_Parity->setCurrentText("None");
    m_ui->comboBox_Stopbits->setCurrentText("1");
    m_ui->comboBox_Flowcontrol->setCurrentText("None");
}

void MainWindow::serial_updateSetting() {

    m_serial->setPortName(m_ui->comboBox_Comport->currentText());
    m_serial->setBaudRate(static_cast<QSerialPort::BaudRate>
                          (m_ui->comboBox_Baudrate->itemData(m_ui->comboBox_Baudrate->currentIndex()).toInt()));
    m_serial->setDataBits(static_cast<QSerialPort::DataBits>
                          (m_ui->comboBox_Databits->itemData(m_ui->comboBox_Databits->currentIndex()).toInt()));
    m_serial->setParity(static_cast<QSerialPort::Parity>
                          (m_ui->comboBox_Parity->itemData(m_ui->comboBox_Parity->currentIndex()).toInt()));
    m_serial->setStopBits(static_cast<QSerialPort::StopBits>
                          (m_ui->comboBox_Stopbits->itemData(m_ui->comboBox_Stopbits->currentIndex()).toInt()));
    m_serial->setFlowControl(static_cast<QSerialPort::FlowControl>
                          (m_ui->comboBox_Flowcontrol->itemData(m_ui->comboBox_Flowcontrol->currentIndex()).toInt()));
//    qDebug() << m_serial->portName() << m_serial->baudRate();
}

void MainWindow::serial_updatePortName() {
    const auto port_info = QSerialPortInfo::availablePorts();
    int count_port = port_info.count();
    int count_item = m_ui->comboBox_Comport->count();

    if(count_port < count_item) {
        for(int i = count_port; i < count_item; i++) {
            m_ui->comboBox_Comport->removeItem(i);
        }
        count_item = count_port;
    } else if(count_port > count_item) {
        for(int i = count_item; i < count_port; i++) {
            m_ui->comboBox_Comport->addItem(port_info.at(i).portName());
        }
    }
    for(int i = 0 ; i < count_item; i ++) {
        if(port_info.at(i).portName() != m_ui->comboBox_Comport->itemText(i)) {
            m_ui->comboBox_Comport->removeItem(i);
            m_ui->comboBox_Comport->addItem(port_info.at(i).portName());
        }
    }
}

void MainWindow::statusBar_Message(const QString &message) {
    m_status->setText(message);
}

void MainWindow::serial_handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), m_serial->errorString());
        serial_closePort();
    }
}

void MainWindow::serial_closePort() {
    if(m_serial->isOpen()) {
        m_serial->close();
        m_ui->groupBox_SerialSetting->setEnabled(true);
        m_ui->pushButton_Serial_Connect->setText("Connect");
        m_ui->pushButton_Serial_Default->setEnabled(true);
        timer_serial->start(1000);
        statusBar_Message(tr("Disconnected"));
    } else {
        QMessageBox::critical(this, tr("Error"), m_serial->errorString());
        statusBar_Message(tr("Disconnected"));
    }
}

void MainWindow::serial_openPort() {
    if(m_serial->open(QIODevice::ReadWrite)) {
        m_ui->groupBox_SerialSetting->setEnabled(false);
        m_ui->pushButton_Serial_Connect->setText("Disconnect");
        m_ui->pushButton_Serial_Default->setEnabled(false);
        timer_serial->stop();
        statusBar_Message(tr("Connected to %1 : %2, %3, %4, %5, %6")
                          .arg(m_ui->comboBox_Comport->currentText()).arg(m_ui->comboBox_Baudrate->currentText())
                          .arg(m_ui->comboBox_Parity->currentText()).arg(m_ui->comboBox_Parity->currentText())
                          .arg(m_ui->comboBox_Stopbits->currentText()).arg(m_ui->comboBox_Flowcontrol->currentText()));
    } else {
        QMessageBox::critical(this, tr("Error"), m_serial->errorString());
        statusBar_Message(tr("Open error"));
    }
}

void MainWindow::serial_write(const QByteArray &data) {
    if(m_serial->isOpen()) {
        m_serial->write(data);
    } else {
        QMessageBox::critical(this, tr("Error"), tr("No device connected"));
    }
}

void MainWindow::serial_read() {
    if(m_serial->isOpen()) {
        const QByteArray data = m_serial->readAll();
        const QString string = QString::fromStdString(data.toStdString());
        logs_write(string, Qt::blue);
    } else {
        QMessageBox::critical(this, tr("Error"), tr("No device connected"));
    }
}

void MainWindow::logs_write(const QString &message, const QColor &c) {
    m_ui->textEdit->setTextColor(c);
    m_ui->textEdit->insertPlainText(message);
    m_ui->textEdit->insertPlainText("\n");
}

void MainWindow::logs_clear() {
    m_ui->textEdit->clear();
}

void MainWindow::on_pushButton_Serial_Default_clicked()
{
    serial_setDefault();
    serial_updateSetting();
}

void MainWindow::on_pushButton_Serial_Connect_clicked()
{
    if(m_ui->pushButton_Serial_Connect->text() == "Connect") {
        serial_openPort();
    } else {
        serial_closePort();
    }

}

void MainWindow::on_pushButton_Test_clicked()
{
    const QString text = "nam dep trai";
    serial_write(text.toLocal8Bit());

}

void MainWindow::on_pushButton_Camera_Connect_clicked()
{
    if( m_ui->pushButton_Camera_Connect->text() == "Connect") {
        camera_openCamera();
    } else {
        camera_closeCamera();
    }
}
