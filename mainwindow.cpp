#include "mainwindow.h"
#include "ui_mainwindow.h"
#define COMBOBOX_NUM    (6)
#define CHECKBOX_NUM    (7)
#define PARA_NUM        (4)
#define CAMERA_PARA_NUM (4)
const char* comboBox_Name[COMBOBOX_NUM] = {"comboBox_Comport",
                                           "comboBox_Baudrate",
                                           "comboBox_Databits",
                                           "comboBox_Parity",
                                           "comboBox_Stopbits",
                                           "comboBox_Flowcontrol"};

const char* checkBox_Name[CHECKBOX_NUM] = {"checkBox_SetPos",
                                           "checkBox_SetWidth",
                                           "checkBox_SetHome",
                                           "checkBox_SetDuty",
                                           "checkBox_SetPosNAng",
                                           "checkBox_SetTime",
                                           "checkBox_Save"};

const char* para_Name[PARA_NUM] = {"Para1",
                                   "Para2",
                                   "Para3",
                                   "Para4"};

const char* cameraPara_Name[CAMERA_PARA_NUM] = {"alpha",
                                                "beta",
                                                "gamma",
                                                "BinaryThresh"};

MainWindow::MainWindow(QWidget *parent):QMainWindow(parent),
    m_ui(new Ui::MainWindow),
    m_serial(new RobotControll(this)),
    m_status(new QLabel)
{
    m_ui->setupUi(this);
    m_ui->statusBar->addWidget(m_status);
    serial_init();
    camera_init();
}

MainWindow::~MainWindow()
{
    delete m_ui;
    delete m_serial;
    delete m_status;
    delete timer_camera_comboBox;
    delete timer_serial_comboBox;
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
    m_status->setText(tr("No Robot Device"));
    for(int i = 0; i < COMBOBOX_NUM; i++) {
        connect(this->findChild<QComboBox*>(comboBox_Name[i]), &QComboBox::currentTextChanged,
                this, &MainWindow::serial_updateSetting, Qt::QueuedConnection);
    }

    connect(m_serial, &QSerialPort::errorOccurred, this, &MainWindow::serial_handleError);
    connect(m_ui->pushButton_LogsClear, &QPushButton::clicked, this, &MainWindow::logs_clear);
    connect(m_ui->pushButton_Request, &QPushButton::clicked, this, &MainWindow::manual_checkPara_setCommand);

    for(int i = 0; i < CHECKBOX_NUM; i++) {
        connect(this->findChild<QCheckBox*>(checkBox_Name[i]),QOverload<bool>::of(&QCheckBox::clicked),
                this,  &MainWindow::manual_checkBox_event);
    }
    for(int i = 0; i < PARA_NUM; i ++) {
        this->findChild<QLabel*>(tr("label_%1").arg(para_Name[i]))->hide();
        this->findChild<QTextEdit*>(tr("textEdit_%1").arg(para_Name[i]))->hide();
    }

    m_ui->label_ManualExamplePara->setText("");
    timer_serial_comboBox = new QTimer(this);
    connect(timer_serial_comboBox, &QTimer::timeout, this, &MainWindow::serial_updatePortName);
    timer_serial_comboBox->start(1000);
}

void MainWindow::serial_setDefault() {
    const char* defaultText[5] = {"115200", "8", "None", "1", "None"};
    for(int i = 0; i < COMBOBOX_NUM - 1; i ++) {
        this->findChild<QComboBox*>(comboBox_Name[i+1])->setCurrentText(defaultText[i]);
    }
}

void MainWindow::serial_openPort() {
    if(m_serial->open(QIODevice::ReadWrite)) {
        timer_serial_comboBox->stop();
        m_ui->groupBox_SerialSetting->setEnabled(false);
        m_ui->pushButton_Serial_Default->setEnabled(false);
        m_ui->pushButton_Serial_Connect->setText("Disconnect");
        m_status->setText(tr("Connected to %1 : %2, %3, %4, %5, %6")
                          .arg(m_ui->comboBox_Comport->currentText())
                          .arg(m_ui->comboBox_Baudrate->currentText())
                          .arg(m_ui->comboBox_Parity->currentText())
                          .arg(m_ui->comboBox_Parity->currentText())
                          .arg(m_ui->comboBox_Stopbits->currentText())
                          .arg(m_ui->comboBox_Flowcontrol->currentText()));
    } else {
        QMessageBox::critical(this, tr("Error"), m_serial->errorString());
        m_status->setText(tr("Open error"));
    }
}

void MainWindow::serial_closePort() {
    if(m_serial->isOpen()) {
        m_serial->close();
        m_ui->groupBox_SerialSetting->setEnabled(true);
        m_ui->pushButton_Serial_Connect->setText("Connect");
        m_ui->pushButton_Serial_Default->setEnabled(true);
        timer_serial_comboBox->start(1000);
        m_status->setText(tr("No Robot Device"));
    } else {
        QMessageBox::critical(this, tr("Error"), m_serial->errorString());
        m_status->setText(tr("No Robot Device"));
    }
}

void MainWindow::logs_write(QString message, QColor c) {
    if( message.isEmpty() || message.isNull() ) {
        M_DEBUG("log write error");
        return;
    }
    m_ui->textEdit_logs->setTextColor(c);
    m_ui->textEdit_logs->insertPlainText(message);
    m_ui->textEdit_logs->insertPlainText("\n");
}

void MainWindow::manual_checkBox_event(bool checked) {
    QCheckBox *checkbox = (QCheckBox*)sender();
    int para_num_show = 0;
    if(checked) {
        for(int i = 0; i < CHECKBOX_NUM; i++) {
            this->findChild<QCheckBox*>(checkBox_Name[i])->setEnabled(false);
        }
        m_ui->pushButton_Request->setEnabled(true);

        if(checkbox == m_ui->checkBox_SetPos) {
            m_ui->label_Para1->setText("X    ");
            m_ui->label_Para2->setText("Y    ");
            m_ui->label_Para3->setText("Z    ");
            m_ui->label_ManualExamplePara->setText("X, Y, Z: \"10.2\" ");
            para_num_show = 3;

        } else if(checkbox == m_ui->checkBox_SetWidth) {
            m_ui->label_Para1->setText("Width");
            m_ui->label_ManualExamplePara->setText("Width: 3.0 -> 6.0");
            para_num_show = 1;

        } else if(checkbox == m_ui->checkBox_SetDuty) {
            m_ui->label_Para1->setText("Duty");
            m_ui->label_Para2->setText("Channel");
            m_ui->label_ManualExamplePara->setText("Duty: 1000 -> 2000, Channel: 1 -> 6");
            para_num_show = 2;

        } else if(checkbox == m_ui->checkBox_SetHome) {

            para_num_show = 0;
        } else if(checkbox == m_ui->checkBox_SetPosNAng) {
            m_ui->label_Para1->setText("X    ");
            m_ui->label_Para2->setText("Y    ");
            m_ui->label_Para3->setText("Z    ");
            m_ui->label_Para4->setText("Angle");
            m_ui->label_ManualExamplePara->setText("X, Y, Z: \"10.2\" ; Angle: 0->90");
            para_num_show = 4;

        } else if(checkbox == m_ui->checkBox_SetTime) {
            m_ui->label_Para1->setText("Time(ms)");
            m_ui->label_ManualExamplePara->setText("Time: 1000 -> 5000");
            para_num_show = 1;

        } else if(checkbox == m_ui->checkBox_Save) {
            para_num_show = 0;
        }
        checkbox->setEnabled(true);
        for(int i = 0; i < para_num_show; i ++) {
            this->findChild<QLabel*>(tr("label_%1").arg(para_Name[i]))->show();
            this->findChild<QTextEdit*>(tr("textEdit_%1").arg(para_Name[i]))->show();
        }
    } else {
        for(int i = 0; i < CHECKBOX_NUM; i ++) {
            this->findChild<QCheckBox*>(checkBox_Name[i])->setEnabled(true);
        }
        for(int i = 0; i < PARA_NUM; i ++) {
            this->findChild<QLabel*>(tr("label_%1").arg(para_Name[i]))->hide();
            this->findChild<QTextEdit*>(tr("textEdit_%1").arg(para_Name[i]))->hide();
        }
        m_ui->pushButton_Request->setEnabled(false);
        m_ui->label_ManualExamplePara->setText("");
    }
}

void MainWindow::manual_checkPara_setCommand() {
    if(m_ui->checkBox_SetPos->isChecked()) {
        bool isDouble_1, isDouble_2, isDouble_3;
        m_ui->textEdit_Para1->toPlainText().toDouble(&isDouble_1);
        m_ui->textEdit_Para2->toPlainText().toDouble(&isDouble_2);
        m_ui->textEdit_Para3->toPlainText().toDouble(&isDouble_3);
        if(isDouble_1 & isDouble_2 & isDouble_3) {
            if( m_serial->setCommand( RobotControll::SetPosition, 1000, tr("%1 %2 %3")
                                      .arg(m_ui->textEdit_Para1->toPlainText())
                                      .arg(m_ui->textEdit_Para2->toPlainText())
                                      .arg(m_ui->textEdit_Para3->toPlainText())) == false ) {
                return;
            }
        } else {
            QMessageBox::critical(this, tr("Error"), tr("All parameter must is double"));
            return;
        }
    } else if(m_ui->checkBox_SetWidth->isChecked()) {
        bool isDouble;
        m_ui->textEdit_Para1->toPlainText().toDouble(&isDouble);
        if(isDouble) {
            if( m_serial->setCommand( RobotControll::SetWidth, 1000, tr("%1")
                                      .arg(m_ui->textEdit_Para1->toPlainText())) == false ) {
                return;
            }
        } else {
            QMessageBox::critical(this, tr("Error"), tr("All parameter must is double"));
            return;
        }
    } else if(m_ui->checkBox_SetDuty->isChecked()) {
        bool isInt_1, isInt_2;
        m_ui->textEdit_Para1->toPlainText().toInt(&isInt_1);
        m_ui->textEdit_Para2->toPlainText().toInt(&isInt_2);
        if(isInt_1 & isInt_2) {
            if( m_serial->setCommand( RobotControll::SetDuty, 1000, tr("%1 %2")
                                      .arg(m_ui->textEdit_Para1->toPlainText())
                                      .arg(m_ui->textEdit_Para2->toPlainText())) == false ) {
                return;
            }
        } else {
            QMessageBox::critical(this, tr("Error"), tr("All parameter must is int"));
            return;
        }

    } else if(m_ui->checkBox_SetHome->isChecked()) {

        if(m_serial->setCommand(RobotControll::SetHome, 1000, tr("")) == false ) {
            return;
        }
    } else if(m_ui->checkBox_SetPosNAng->isChecked()) {
        bool isDouble_1, isDouble_2, isDouble_3, isInt;
        m_ui->textEdit_Para1->toPlainText().toDouble(&isDouble_1);
        m_ui->textEdit_Para2->toPlainText().toDouble(&isDouble_2);
        m_ui->textEdit_Para3->toPlainText().toDouble(&isDouble_3);
        m_ui->textEdit_Para4->toPlainText().toInt(&isInt);
        if(isDouble_1 & isDouble_2 & isDouble_3 & isInt) {
            if( m_serial->setCommand( RobotControll::SetPositionWithArg, 1000, tr("%1 %2 %3 %4")
                                      .arg(m_ui->textEdit_Para1->toPlainText())
                                      .arg(m_ui->textEdit_Para2->toPlainText())
                                      .arg(m_ui->textEdit_Para3->toPlainText())
                                      .arg(m_ui->textEdit_Para4->toPlainText())) == false ) {
                return;
            }
        } else {
            QMessageBox::critical(this, tr("Error"), tr("x,y,z parameter must is double, angle is int"));
            return;
        }
    } else if(m_ui->checkBox_SetTime->isChecked()) {
        bool isInt;
        m_ui->textEdit_Para1->toPlainText().toDouble(&isInt);
        if(isInt) {
            if( m_serial->setCommand( RobotControll::SetTime, 1000, tr("%1")
                                      .arg(m_ui->textEdit_Para1->toPlainText())) == false ) {
                return;
            }
        } else {
            QMessageBox::critical(this, tr("Error"), tr("All parameter must is Int"));
            return;
        }
    } else if(m_ui->checkBox_Save->isChecked()) {

        if(m_serial->setCommand(RobotControll::Save, 1000, tr("")) == false ) {
            return;
        }
    } else {
        QMessageBox::critical(this, tr("Error"), tr("No request to send"));
    }
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

void MainWindow::serial_handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), m_serial->errorString());
        serial_closePort();
    }
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

// camera function
void MainWindow::camera_init()
{
    camera_updateDevice();
    timer_imgproc = new QTimer(this);
    connect(timer_imgproc, &QTimer::timeout, this, &MainWindow::cv_timeout);
    connect(this, QOverload<bool>::of(&MainWindow::cv_signalShow), this, &MainWindow::cv_show);
    connect(m_ui->label_Camera_show, &QLabel_custom::mouseReleased, this, &MainWindow::cv_getROI);

    connect(m_ui->pushButton_Calib,&QPushButton::clicked , this, &MainWindow::cv_calib);
    connect(m_ui->pushButton_Run, &QPushButton::clicked, this, &MainWindow::cv_autoRun);
    connect(m_ui->pushButton_SaveObject, &QPushButton::clicked, this, &MainWindow::cv_saveImageFromROI);
    connect(m_ui->pushButton_SaveBaseArea, &QPushButton::clicked, this, &MainWindow::cv_saveImageFromROI);

    timer_camera_comboBox = new QTimer(this);
    connect(timer_camera_comboBox, &QTimer::timeout, this, &MainWindow::camera_updateDevice);
    timer_camera_comboBox->start(MAIN_TIMER_REFRESH);

    for(int i = 0; i < CAMERA_PARA_NUM; i++) {
        connect(this->findChild<QSlider*>(tr("horizontalSlider_%1").arg(cameraPara_Name[i])),
                QOverload<int>::of(&QSlider::valueChanged), this, &MainWindow::dip_sliderChanged);
        connect(this->findChild<QDoubleSpinBox*>(tr("doubleSpinBox_%1").arg(cameraPara_Name[i])),
                &QDoubleSpinBox::editingFinished, this, &MainWindow::dip_spinBoxEditingFinished);
    }

    connect(m_ui->checkBox_EnableHSV, QOverload<bool>::of(&QCheckBox::clicked), this, &MainWindow::dip_checkBoxEnableClicked);
    connect(m_ui->checkBox_EnableSUFT, QOverload<bool>::of(&QCheckBox::clicked), this, &MainWindow::dip_checkBoxEnableClicked);
    connect(m_ui->checkBox_ConvertImage, QOverload<bool>::of(&QCheckBox::clicked), this, &MainWindow::dip_checkBoxEnableClicked);
    connect(m_ui->checkBox_NonBaseImage, QOverload<bool>::of(&QCheckBox::clicked), this, &MainWindow::dip_checkBoxEnableClicked);

    double _alpha, _beta, _gamma, _threshbinary;
    Scalar _hsv_high, _hsv_low;
    ImageProcess::getParameterFromFile(_alpha, _beta, _gamma, _threshbinary, _hsv_high, _hsv_low);
    double value[CAMERA_PARA_NUM] = {_alpha, _beta, _gamma, _threshbinary};
    //                                     _hsv_high[0], _hsv_low[0], _hsv_high[1],
    //                                     _hsv_low[1], _hsv_high[2], _hsv_low[2]};
    for(int i = 0; i < CAMERA_PARA_NUM; i++) {
        this->findChild<QDoubleSpinBox*>(tr("doubleSpinBox_%1").arg(cameraPara_Name[i]))->setValue(value[i]);
        pre_setSlider(this->findChild<QDoubleSpinBox*>(tr("doubleSpinBox_%1").arg(cameraPara_Name[i])),
                      this->findChild<QSlider*>(tr("horizontalSlider_%1").arg(cameraPara_Name[i])));
    }

    auto_run_timer = new QTimer(this);
    connect(auto_run_timer, &QTimer::timeout, this, &MainWindow::autoGrabObject);
}

void MainWindow::camera_openCamera()
{
    m_camera.open(m_ui->comboBox_CameraDevice->currentIndex());
    m_camera.init();
    if(m_camera.isOpened()) {
        m_ui->comboBox_CameraDevice->setEnabled(false);
        m_ui->pushButton_Camera_Connect->setText(tr("Disconnect"));
        timer_camera_comboBox->stop();
        timer_imgproc->start(DIP_MAIN_FPS);
    } else {
        QMessageBox::critical(this, tr("Critical Error"), tr("error open camera"));
    }
}

void MainWindow::camera_closeCamera()
{
    m_camera.denit();
    m_camera.release();
    if(m_camera.isOpened()) {
        QMessageBox::critical(this, tr("Critical Error"), tr("error close camera"));
    } else {
        timer_imgproc->stop();
        timer_camera_comboBox->start(MAIN_TIMER_REFRESH);
        m_ui->comboBox_CameraDevice->setEnabled(true);
        m_ui->pushButton_Camera_Connect->setText(tr("Connect"));
    }
}

void MainWindow::camera_updateDevice()
{
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

void MainWindow::cv_qtshow(Mat img, QImage::Format format)
{
    Mat temp;
    cvtColor(img, temp, COLOR_BGR2RGB);
    QImage* qimage = new QImage(temp.data, temp.cols, temp.rows, temp.step, format);
    m_ui->label_Camera_show->setFixedSize(qimage->size());
    m_ui->label_Camera_show->setPixmap(QPixmap::fromImage(*qimage));

    Debug::_delete(temp);
    delete qimage;
}

// emit signal to show image from input.
// can use dynamic flag to show image for loop.
void MainWindow::cv_debugImage( Mat image)
{
    timer_imgproc->stop();
    if(image.empty()) {
        M_DEBUG("input image debug empty");
        return;
    } else {
        image.assignTo(cv_image);
    }
    emit cv_signalShow(false);
}

void MainWindow::cv_getROI()
{
    QRect rect = m_ui->label_Camera_show->getRect();
    cv_debugImage(m_camera.getMatFromQPixmap(m_ui->label_Camera_show->grab(rect)));
}

void MainWindow::cv_saveImageFromROI()
{
    QPushButton *button = (QPushButton*)sender();
    if(cv_image.empty()) {
        M_DEBUG("cv image is empty");
        return;
    }
    if(button == m_ui->pushButton_SaveObject) {
        imwrite(ImageProcess::getNode(ImageProcess::PathObjectSave), cv_image);
        QMessageBox::information(this, tr("Object"), tr("Save ok"));
    } else if(button == m_ui->pushButton_SaveBaseArea) {
        imwrite(ImageProcess::getNode(ImageProcess::PathBaseAreaSave), cv_image);
        m_camera.setBase();
        QMessageBox::information(this, tr("Base"), tr("Save ok"));
    }

    emit cv_signalShow(true);
}

void MainWindow::cv_timeout()
{
    emit cv_signalShow(true);
}

void MainWindow::pre_setSlider(QDoubleSpinBox *spinbox, QSlider *slider)
{
    double value = spinbox->value();
    int position = (int)((double)slider->minimum() + (value - spinbox->minimum())*
                         (double)(slider->maximum()-slider->minimum())/(spinbox->maximum() - spinbox->minimum()));
    slider->setSliderPosition(position);
}

void MainWindow::pre_setSpinBox(QSlider *slider, QDoubleSpinBox *spinbox)
{
    double value = (double)slider->value();
    double spinbox_value = (double)(spinbox->minimum() + (value - (double)slider->minimum())*
                                    (spinbox->maximum() - spinbox->minimum())/
                                    ((double)slider->maximum() - (double)slider->minimum()));
    spinbox->setValue(spinbox_value);
}

void MainWindow::cv_show(bool dynamic) {
    // stop timer timeout
    if(dynamic) {
        timer_imgproc->start(DIP_MAIN_FPS);
        Mat show_img;
        m_camera.process();
        m_camera.getImage(show_img);
        cv_qtshow(show_img, QImage::Format_RGB888);
        Debug::_delete(show_img);
    } else {
        timer_imgproc->stop();
        if(cv_image.empty()) {
            M_DEBUG("cv_image is empty");
            return;
        }
        cv_qtshow(cv_image, QImage::Format_RGB888);
    }
}

void MainWindow::cv_calib()
{
    dip_default();
    m_ui->pushButton_Calib->setEnabled(false);
    Size patternSize(8,6); //interior number of corners
    vector<vector<Point3f>> listRealPoints;
    vector<vector<Point2f>> listImagePoints;

    // for loop and check to rotate image
    Mat grayImage;
    for(size_t i = 0; i < 1 ; i++ ) {
        QMessageBox::information(this, tr("Calib"), tr("move pattern (%1/10 images)").arg(i+1));
        grayImage = m_camera.getImageFromCamera(COLOR_BGR2GRAY);
        if( m_camera.getPattern2CalibCamera(grayImage, (float)0.025, patternSize,
                                            listImagePoints, listRealPoints) == false ) {
            m_ui->pushButton_Calib->setEnabled(true);
            Debug::_delete(listImagePoints, listRealPoints, grayImage);
            return;
        }
    }
    m_camera.calibCamera(listImagePoints, listRealPoints, grayImage.size());

    vector<Point2f> imagePoints, realPoints;
    if( m_camera.getPattern2CalibRobot(grayImage, (float)25.0, patternSize,
                                       imagePoints, realPoints) == false ) {
        m_ui->pushButton_Calib->setEnabled(true);
        Debug::_delete(listImagePoints, listRealPoints, grayImage, imagePoints, realPoints);
        return;
    }
    m_camera.calibRobot(imagePoints, realPoints);

    Debug::_delete(listImagePoints, listRealPoints, grayImage, imagePoints, realPoints);
    m_ui->pushButton_Calib->setEnabled(true);
}

void MainWindow::cv_autoRun()
{
    if(m_ui->pushButton_Run->text() == "Run") {
        m_ui->pushButton_Run->setText(tr("Cancel"));
        auto_run_timer->start(20);
    } else {
        m_ui->pushButton_Run->setText(tr("Run"));
        auto_run_timer->stop();
    }
}

void MainWindow::autoGrabObject()
{
    disconnect(auto_run_timer, &QTimer::timeout, this, &MainWindow::autoGrabObject);
    Filter::Object_t object = {Point2f(0,0), 0, 0};
    if(m_camera.getObject(object, -1) == false) {
        connect(auto_run_timer, &QTimer::timeout, this, &MainWindow::autoGrabObject);
        return;
    }
    if((object.center.x == 0 && object.center.y == 0) || object.radius_img == 0) {
        M_DEBUG("error radius or center is zero");
        connect(auto_run_timer, &QTimer::timeout, this, &MainWindow::autoGrabObject);
        return;
    }

    Point2f real_center, real_base;
    double real_width;
    ImageProcess::toReal(object.center, real_center);
    ImageProcess::toReal(object.radius_img, real_width);
    if(real_width*2.0/10.0 > 35.0) {
        M_DEBUG("non object");
        m_camera.clearFilter();
        connect(auto_run_timer, &QTimer::timeout, this, &MainWindow::autoGrabObject);
        return;
    }
    ImageProcess::toReal(m_camera.getBaseCenter(), real_base);
    //    qDebug() << tr("x:%1, y:%2").arg(center_real.x).arg(center_real.y);
    m_serial->setWidthNPosition(real_center, 2000, real_width*2.0/10.0, real_base);
    m_camera.clearFilter();
    connect(auto_run_timer, &QTimer::timeout, this, &MainWindow::autoGrabObject);
}

void MainWindow::on_pushButton_Camera_Connect_clicked()
{
    if( m_ui->pushButton_Camera_Connect->text() == "Connect") {
        camera_openCamera();
    } else {
        camera_closeCamera();
    }
}

void MainWindow::on_pushButton_ShowCamera_clicked()
{
    m_camera.setMode(ImageProcess::ModeNull);
    dip_default();
    emit cv_signalShow(true);
}

void MainWindow::dip_sliderChanged(int value)
{
    Q_UNUSED(value);
    QSlider *slider = (QSlider*)sender();
    for(int i = 0; i < CAMERA_PARA_NUM; i ++) {
        if(slider == this->findChild<QSlider*>(tr("horizontalSlider_%1").arg(cameraPara_Name[i]))) {
            pre_setSpinBox(slider, this->findChild<QDoubleSpinBox*>(tr("doubleSpinBox_%1")
                                                                    .arg(cameraPara_Name[i])));
            break;
        }
    }

    //    Scalar _hsv_max(m_ui->doubleSpinBox_Hmax->value(),
    //                    m_ui->doubleSpinBox_Smax->value(),
    //                    m_ui->doubleSpinBox_Vmax->value());
    //    Scalar _hsv_min(m_ui->doubleSpinBox_Hmin->value(),
    //                    m_ui->doubleSpinBox_Smin->value(),
    //                    m_ui->doubleSpinBox_Vmin->value());
    Scalar _hsv_max(0, 0, 0);
    Scalar _hsv_min(0, 0, 0);
    m_camera.setPreProcessParameter(m_ui->doubleSpinBox_gamma->value(),
                                    m_ui->doubleSpinBox_alpha->value(),
                                    m_ui->doubleSpinBox_beta->value(),
                                    m_ui->doubleSpinBox_BinaryThresh->value(),
                                    _hsv_max, _hsv_min);
}

void MainWindow::dip_spinBoxEditingFinished()
{
    QDoubleSpinBox *spinbox = (QDoubleSpinBox*)sender();
    for(int i = 0; i < CAMERA_PARA_NUM; i ++) {
        if(spinbox == this->findChild<QDoubleSpinBox*>(tr("doubleSpinBox_%1").arg(cameraPara_Name[i]))) {
            pre_setSlider(spinbox, this->findChild<QSlider*>(tr("horizontalSlider_%1").arg(cameraPara_Name[i])));
            break;
        }
    }
}

void MainWindow::dip_checkBoxEnableClicked(bool checked)
{
    QCheckBox *checkbox = (QCheckBox*)sender();
    if(m_ui->pushButton_Run->text() == "Cancel") {
        emit m_ui->pushButton_Run->clicked();
    }
    if(checked) {
        m_ui->groupBox_Basic->setEnabled(false);
        m_ui->groupBox_SUFT->setEnabled(false);
        m_ui->checkBox_ConvertImage->setEnabled(false);
        m_ui->checkBox_NonBaseImage->setEnabled(false);

        if(checkbox == m_ui->checkBox_EnableHSV) {
            m_ui->groupBox_Basic->setEnabled(true);
            m_camera.setMode(m_camera.ModeBasicProcessing);

        } else if(checkbox == m_ui->checkBox_EnableSUFT) {
            m_ui->groupBox_SUFT->setEnabled(true);
            m_camera.setMode(m_camera.ModeSUFT);
        } else if(checkbox == m_ui->checkBox_ConvertImage) {
            m_ui->checkBox_ConvertImage->setEnabled(true);
            m_camera.setMode(m_camera.ModeShowConvertImage);
        } else if(checkbox == m_ui->checkBox_NonBaseImage) {
            m_ui->checkBox_NonBaseImage->setEnabled(true);
            m_camera.setMode(m_camera.ModeNonBase);
        }
    } else {
        m_ui->groupBox_Basic->setEnabled(true);
        m_ui->groupBox_SUFT->setEnabled(true);
        m_ui->checkBox_ConvertImage->setEnabled(true);
        m_ui->checkBox_NonBaseImage->setEnabled(true);
        m_camera.setBase();
        m_camera.setMode(m_camera.ModeNull);
        m_camera.clearFilter();
    }
}

void MainWindow::dip_default()
{
    m_ui->groupBox_Basic->setEnabled(true);
    m_ui->groupBox_SUFT->setEnabled(true);

    m_ui->checkBox_ConvertImage->setEnabled(true);
    m_ui->checkBox_NonBaseImage->setEnabled(true);
    m_ui->checkBox_EnableHSV->setEnabled(true);
    m_ui->checkBox_EnableSUFT->setEnabled(true);

    m_ui->checkBox_ConvertImage->setChecked(false);
    m_ui->checkBox_NonBaseImage->setChecked(false);
    m_ui->checkBox_EnableHSV->setChecked(false);
    m_ui->checkBox_EnableSUFT->setChecked(false);
}
