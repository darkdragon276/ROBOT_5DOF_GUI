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
    delete timer_camera_comboBox;
    delete timer_serial_comboBox;
}

void MainWindow::camera_openCamera() {
    m_camera.open(m_ui->comboBox_CameraDevice->currentIndex());
    if(m_camera.isOpened()) {
        m_ui->comboBox_CameraDevice->setEnabled(false);
        m_ui->pushButton_Camera_Connect->setText(tr("Disconnect"));
        timer_camera_comboBox->stop();
        timer_imgproc->start(DELAY_CAPTURE_20MS);
    } else {
        QMessageBox::critical(this, tr("Critical Error"), tr("error open camera"));
    }
}

void MainWindow::camera_closeCamera() {
    m_camera.release();
    if(m_camera.isOpened()) {
        QMessageBox::critical(this, tr("Critical Error"), tr("error close camera"));
    } else {
        timer_imgproc->stop();
        timer_camera_comboBox->start(1000);
        m_ui->comboBox_CameraDevice->setEnabled(true);
        m_ui->pushButton_Camera_Connect->setText(tr("Connect"));
    }
}

// init timer of comboBox
void MainWindow::timer_init() {
    timer_serial_comboBox = new QTimer(this);
    connect(timer_serial_comboBox, &QTimer::timeout, this, &MainWindow::serial_updatePortName);
    timer_serial_comboBox->start(1000);

    timer_camera_comboBox = new QTimer(this);
    connect(timer_camera_comboBox, &QTimer::timeout, this, &MainWindow::camera_updateDevice);
    timer_camera_comboBox->start(1000);
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

// serial function
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
    statusBar_Message(tr("No Robot Device is Connected"));

    connect(m_ui->comboBox_Comport, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );
    connect(m_ui->comboBox_Baudrate, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );
    connect(m_ui->comboBox_Databits, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );
    connect(m_ui->comboBox_Parity, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );
    connect(m_ui->comboBox_Stopbits, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );
    connect(m_ui->comboBox_Flowcontrol, &QComboBox::currentTextChanged, this, &MainWindow::serial_updateSetting );

    connect(m_serial, &QSerialPort::errorOccurred, this, &MainWindow::serial_handleError);
    connect(m_serial, &QSerialPort::readyRead, this, &MainWindow::serial_read);
    connect(m_ui->pushButton_LogsClear, &QPushButton::clicked, this, &MainWindow::logs_clear);

    connect(m_ui->checkBox_SetPos, QOverload<bool>::of(&QCheckBox::clicked), this,  &MainWindow::manual_checkBox_event);
    connect(m_ui->checkBox_SetWidth, QOverload<bool>::of(&QCheckBox::clicked), this,  &MainWindow::manual_checkBox_event);
    connect(m_ui->checkBox_SetHome, QOverload<bool>::of(&QCheckBox::clicked), this,  &MainWindow::manual_checkBox_event);
    connect(m_ui->checkBox_SetDuty, QOverload<bool>::of(&QCheckBox::clicked), this,  &MainWindow::manual_checkBox_event);
    connect(m_ui->checkBox_Save, QOverload<bool>::of(&QCheckBox::clicked), this,  &MainWindow::manual_checkBox_event);

    m_ui->label_Para1->hide();
    m_ui->textEdit_Para1->hide();
    m_ui->label_Para2->hide();
    m_ui->textEdit_Para2->hide();
    m_ui->label_Para3->hide();
    m_ui->textEdit_Para3->hide();
    m_ui->label_ManualExamplePara->setText("");
}

void MainWindow::manual_checkBox_event(bool checked) {
    QCheckBox *checkbox = (QCheckBox*)sender();
    if(checked) {
        m_ui->checkBox_SetPos->setEnabled(false);
        m_ui->checkBox_SetWidth->setEnabled(false);
        m_ui->checkBox_SetHome->setEnabled(false);
        m_ui->checkBox_SetDuty->setEnabled(false);
        m_ui->checkBox_Save->setEnabled(false);

        m_ui->pushButton_Request->setEnabled(true);
        if(checkbox == m_ui->checkBox_SetPos) {

            m_ui->checkBox_SetPos->setEnabled(true);
            m_ui->label_Para1->setText("X    ");
            m_ui->label_Para2->setText("Y    ");
            m_ui->label_Para3->setText("Z    ");

            m_ui->label_Para1->show();
            m_ui->textEdit_Para1->show();
            m_ui->label_Para2->show();
            m_ui->textEdit_Para2->show();
            m_ui->label_Para3->show();
            m_ui->textEdit_Para3->show();

            m_ui->label_ManualExamplePara->setText("X, Y, Z: \"10.2\" ");
        } else if(checkbox == m_ui->checkBox_SetWidth) {

            m_ui->checkBox_SetWidth->setEnabled(true);
            m_ui->label_Para1->setText("Width");

            m_ui->label_Para1->show();
            m_ui->textEdit_Para1->show();

            m_ui->label_ManualExamplePara->setText("Width: 3.0 -> 6.0");
        } else if(checkbox == m_ui->checkBox_SetDuty) {

            m_ui->checkBox_SetDuty->setEnabled(true);
            m_ui->label_Para1->setText("Duty");
            m_ui->label_Para2->setText("Channel");

            m_ui->label_Para1->show();
            m_ui->textEdit_Para1->show();
            m_ui->label_Para2->show();
            m_ui->textEdit_Para2->show();

            m_ui->label_ManualExamplePara->setText("Duty: 1000 -> 2000, Channel: 1 -> 6");
        } else if(checkbox == m_ui->checkBox_SetHome) {

            m_ui->checkBox_SetHome->setEnabled(true);

        } else if(checkbox == m_ui->checkBox_Save) {

            m_ui->checkBox_Save->setEnabled(true);

        }
    } else {
        m_ui->checkBox_SetPos->setEnabled(true);
        m_ui->checkBox_SetWidth->setEnabled(true);
        m_ui->checkBox_SetHome->setEnabled(true);
        m_ui->checkBox_SetDuty->setEnabled(true);
        m_ui->checkBox_Save->setEnabled(true);

        m_ui->pushButton_Request->setEnabled(false);
        m_ui->label_Para1->hide();
        m_ui->textEdit_Para1->hide();
        m_ui->label_Para2->hide();
        m_ui->textEdit_Para2->hide();
        m_ui->label_Para3->hide();
        m_ui->textEdit_Para3->hide();
        m_ui->label_ManualExamplePara->setText("");
    }

}

void MainWindow::manual_checkPara_sendRequest() {
    if(m_ui->checkBox_SetPos->isChecked()) {
        bool isDouble_1, isDouble_2, isDouble_3;
        m_ui->textEdit_Para1->toPlainText().toDouble(&isDouble_1);
        m_ui->textEdit_Para2->toPlainText().toDouble(&isDouble_2);
        m_ui->textEdit_Para3->toPlainText().toDouble(&isDouble_3);
        if(isDouble_1 & isDouble_2 & isDouble_3) {
            send_request(id_command, tr("SETPOS"), tr("%1 %2 %3").arg(m_ui->textEdit_Para1->toPlainText())
                         .arg(m_ui->textEdit_Para2->toPlainText())
                         .arg(m_ui->textEdit_Para3->toPlainText()));
        } else {
            QMessageBox::critical(this, tr("Error"), tr("All parameter must is double"));
            return;
        }
    } else if(m_ui->checkBox_SetWidth->isChecked()) {
        bool isDouble;
        m_ui->textEdit_Para1->toPlainText().toDouble(&isDouble);
        if(isDouble) {
            send_request(id_command, tr("SETWID"), tr("%1").arg(m_ui->textEdit_Para1->toPlainText()));
        } else {
            QMessageBox::critical(this, tr("Error"), tr("All parameter must is double"));
            return;
        }
    } else if(m_ui->checkBox_SetDuty->isChecked()) {
        bool isInt_1, isInt_2;
        m_ui->textEdit_Para1->toPlainText().toInt(&isInt_1);
        m_ui->textEdit_Para2->toPlainText().toInt(&isInt_2);
        if(isInt_1 & isInt_2) {
            send_request(id_command, tr("SETDUTY"), tr("%1 %2").arg(m_ui->textEdit_Para1->toPlainText())
                         .arg(m_ui->textEdit_Para2->toPlainText()));
        } else {
            QMessageBox::critical(this, tr("Error"), tr("All parameter must is int"));
            return;
        }

    } else if(m_ui->checkBox_SetHome->isChecked()) {

        send_request(id_command, tr("SETHOME"), tr(""));

    } else if(m_ui->checkBox_Save->isChecked()) {

        send_request(id_command, tr("SAVE"), tr(""));

    } else {
        QMessageBox::critical(this, tr("Error"), tr("No request to send"));
    }
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
        timer_serial_comboBox->start(1000);
        statusBar_Message(tr("No Robot Device is Connected"));
    } else {
        QMessageBox::critical(this, tr("Error"), m_serial->errorString());
        statusBar_Message(tr("No Robot Device is Connected"));
    }
}

void MainWindow::serial_openPort() {
    if(m_serial->open(QIODevice::ReadWrite)) {
        m_ui->groupBox_SerialSetting->setEnabled(false);
        m_ui->pushButton_Serial_Connect->setText("Disconnect");
        m_ui->pushButton_Serial_Default->setEnabled(false);
        timer_serial_comboBox->stop();
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
        serial_pack(data);
        m_serial->write(m_dataserial);
        logs_write(QString::fromLocal8Bit(m_dataserial), Qt::red);
    } else {
        QMessageBox::critical(this, tr("Error"), tr("No device connected"));
    }
}

void MainWindow::serial_read() {
    if(m_serial->isOpen()) {
        QByteArray data = m_serial->readAll();
        while(data.lastIndexOf(0x7E) != 0) {
            QByteArray temp = data.mid(data.lastIndexOf(0x7E));
            data.remove(data.lastIndexOf(0x7E), data.length());
            serial_unpack(temp);
            logs_write(QString::fromLocal8Bit(m_dataserial), Qt::red);
        }
        serial_unpack(data);
        logs_write(QString::fromLocal8Bit(m_dataserial), Qt::red);
    } else {
        QMessageBox::critical(this, tr("Error"), tr("No device connected"));
    }
}

void MainWindow::send_request(int &idcommand, const QString command, const QString para) {
    idcommand++;
    QByteArray request;
    request.clear();
    if(command.isEmpty()) {
        QMessageBox::critical(this, tr("Critical Error"), tr("command is null/empty"));
        return;
    }
    if(para.isEmpty()) {
        request.append(tr("%1 %2").arg(QString::number(idcommand))
                       .arg(command));
    } else {
        request.append(tr("%1 %2 %3").arg(QString::number(idcommand))
                       .arg(command).arg(para));
    }
    serial_write(request);
}

void MainWindow::serial_pack(const QByteArray &data)
{
    if(data.isNull() || data.isEmpty()) {
        logs_write(tr("data input of pack function is null/empty"), Qt::red);
        return;
    }
    // packing
    m_mutex.lock();
    m_dataserial.clear();
    m_dataserial.append(data);
    int len = m_dataserial.length();
    m_dataserial.push_front((char)0x7E);
    while(len) {
        int i = m_dataserial.length()-len;
        if (m_dataserial.at(i) == (char)0x7D || m_dataserial.at(i) == (char)0x7E ||
                m_dataserial.at(i) == (char)0x7F) {
            char temp = m_dataserial.at(i);
            temp ^= (char)0x02;
            m_dataserial.remove(i, 1);
            m_dataserial.insert(i, temp);
            m_dataserial.insert(i, 0x7D);
        }
        len--;
    }
    m_dataserial.push_back((char)0x7F);
    m_mutex.unlock();
}

void MainWindow::serial_unpack(const QByteArray &data)
{
    if(data.isNull() || data.isEmpty()) {
        logs_write(tr("data input of pack function is null/empty"), Qt::red);
        return;
    }
    if(data.at(0) != 0x7E || data.at(data.length()-1) != 0x7F) {
        logs_write(tr("frame error. not have begin/end character"), Qt::red);
        return;
    }

    m_mutex.lock();
    m_dataserial.clear();
    m_dataserial.append(data);
    m_dataserial.remove(0, 1);
    m_dataserial.remove(m_dataserial.length()-1, 1);
    int len = m_dataserial.length();
    while(len) {
        int i = m_dataserial.length()-len;
        if (m_dataserial.at(i) == (char)0x7D || m_dataserial.at(i) == (char)0x7E ||
                m_dataserial.at(i) == (char)0x7F) {
            char temp = m_dataserial.at(i+1);
            temp ^= (char)0x02;
            m_dataserial.remove(i, 2);
            m_dataserial.insert(i, temp);
            len--;
        }
        len--;
    }
    m_mutex.unlock();
}

void MainWindow::logs_write(const QString &message, const QColor &c) {
    m_ui->textEdit_logs->setTextColor(c);
    m_ui->textEdit_logs->insertPlainText(message);
    m_ui->textEdit_logs->insertPlainText("\n");
}

void MainWindow::logs_clear() {
    m_ui->textEdit_logs->clear();
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

void MainWindow::on_pushButton_Camera_Connect_clicked()
{
    if( m_ui->pushButton_Camera_Connect->text() == "Connect") {
        camera_openCamera();
    } else {
        camera_closeCamera();
    }
}

void MainWindow::on_pushButton_Request_clicked()
{
    //    QByteArray test = QByteArrayLiteral("\x7D\x00\xa4\x42\x51\x00\x7E\x7F");
    //    QByteArray test("nam dep trai");
    //    send_request(id_command, tr("SETPOS"), tr("10.5 5.0 6"));
    //    send_request(id_command, tr("SETWID"), tr("3"));
    manual_checkPara_sendRequest();
}

void MainWindow::on_pushButton_Calib_clicked()
{
    if(m_ui->pushButton_Calib->text() == "Calib") {
        m_ui->pushButton_Calib->setText("Cancel calib");
        imgproc_ctrl_flag = IMGPROC_CALIB;
    } else {
        m_ui->pushButton_Calib->setText("Calib");
        imgproc_ctrl_flag = IMGPROC_SHOW;
    }
}
