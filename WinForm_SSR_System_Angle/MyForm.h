#pragma once
#define _USE_MATH_DEFINES
#include <Windows.h>
#include"math.h"
#include <time.h>
#include "Pt.h"
#include "CTBox.h"
#include <fstream>
#include"cv.h"
#include"highgui.h"

typedef unsigned int uint;
namespace WinForm_SSR_System_Angle {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO;
	using namespace System::IO::Ports;
	using namespace System::Runtime::InteropServices;
	using namespace System::Media;

	using namespace std;
	using namespace cv;
	int LiDAR_Data[722];
	cv::vector<Pt> LIDAR_cooridate;
	CTBox TBox;
	Radar RadarData;
	double PartitionValue = 0;
	vector<Pt>Pt_OldCluster;
	Pt LiDAR_tmpPt = Pt(0, 0);
	Pt left_Radar_bias;
	Pt right_Radar_bias;
	Pt AngleRadar_Point;
	double LIDAR_X_cooridate[361];
	double LIDAR_Y_cooridate[361];

	cv::Size videoSize;
	char RRadarFileName[30] = { 0 };
	char LRadarFileName[30] = { 0 };

	/// <summary>
	/// MyForm 的摘要
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO:  在此加入建構函式程式碼
			//
			int Date = System::DateTime::Now.Day * 10000 + System::DateTime::Now.Hour * 100 + System::DateTime::Now.Minute;
			sprintf(RRadarFileName, ".\\Data\\Passenger%d.txt", Date);
			sprintf(LRadarFileName, ".\\Data\\Driver%d.txt", Date);
			targetDistant = Convert::ToDouble(txBox_targetDistant->Text) * 100;
			PartitionValue = Convert::ToDouble(tBox_Partition->Text) * 100;
			ComPortRefresh();
			timer1->Interval = 20;
			timer1->Start();
			LoadData();
		}

	protected:
		/// <summary>
		/// 清除任何使用中的資源。
		/// </summary>
		~MyForm()
		{
			if (components)
			{

				delete components;
			}
		}

	protected:



	private:
		/// <summary>
		/// 設計工具所需的變數。
		/// </summary>

		time_t t1;
		int counter = 0;
		bool f_getLiDARData = false;
		bool f_getHeader = false;
		bool f_getLRadarBias = false;
		bool f_getRRadarBias = false;
		//SoundPlayer beep_ALL = gcnew SoundPlayer();
		double bsdAngle = 0;
		float AlphaBias;
		double targetDistant;
		uint format = 25;


#pragma region 視窗物件
	private: System::Windows::Forms::TabPage^  tabPage2;
	private: System::Windows::Forms::TabPage^  tabPage1;

	private: System::Windows::Forms::GroupBox^  groupBox2;


	private: System::Windows::Forms::Button^  Btn_RadarA_Connect;
	private: System::Windows::Forms::ComboBox^  cBox_Radar_Angle;
	private: System::Windows::Forms::Button^  Btn_Refresh_Combox;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::ComboBox^  cBox_LiDAR;
	private: System::Windows::Forms::Button^  Btn_LiDAR_Connected;
	private: System::Windows::Forms::Button^  Btn_LiDAR_DisConnect;
	private: System::Windows::Forms::TabControl^  tabControl1;
	private: System::Windows::Forms::Label^  lbBsdAngleT;
	private: System::Windows::Forms::Label^  label2;


	private: System::Windows::Forms::Button^  Btn_RadarAngle_DisConnect;
	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::TabControl^  tabControl2;
	private: System::Windows::Forms::TabPage^  tabPage3;
	private: System::Windows::Forms::TabPage^  tabPage4;
	private: System::Windows::Forms::Label^  tx_LRadarBias_Y;
	private: System::IO::Ports::SerialPort^  serialPort_LiDAR;
	private: System::ComponentModel::IContainer^  components;
	private: System::Windows::Forms::Timer^  timer1;
	private: System::Windows::Forms::Label^  tx_LRadarBias_X;

	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Button^  Btn_LeftBias;
	private: System::Windows::Forms::Label^  tx_RRadarBias_X;
	private: System::Windows::Forms::Label^  tx_RRadarBias_Y;


	private: System::Windows::Forms::Label^  label10;
	private: System::Windows::Forms::Label^  label11;
	private: System::Windows::Forms::Button^  Btn_RightBias;


	private: System::Windows::Forms::GroupBox^  groupBox4;
	private: System::Windows::Forms::ComboBox^  cBox_TBox;

	private: System::Windows::Forms::Button^  Btn_Tbox_Connect;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  Tx_CarSpeed;

	private: System::IO::Ports::SerialPort^  serialPort_Tbox;
	private: System::Windows::Forms::CheckBox^  ckBox_RadarR;
	private: System::Windows::Forms::Label^  Tx_Radar_Mode;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  chart1;
	private: System::Windows::Forms::Button^  Btn_Send_RadarAngle_Cmd;
	private: System::Windows::Forms::Button^  Btn_Tbox_Close;
	private: System::Windows::Forms::Label^  tx_TBox_RAngle;
	private: System::Windows::Forms::Label^  tx_TBox_LAngle;

	private: System::Windows::Forms::GroupBox^  groupBox5;

	private: System::Windows::Forms::Button^  Btn_Radar_Connect;
	private: System::Windows::Forms::ComboBox^  cBox_Radar;
	private: System::IO::Ports::SerialPort^  serialPort_Radar;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::CheckBox^  cBox_Record;
	private: System::Windows::Forms::GroupBox^  groupBox7;
	private: System::Windows::Forms::TextBox^  tBox_Partition;
	private: System::Windows::Forms::GroupBox^  groupBox6;

	private: System::Windows::Forms::Label^  label15;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::TextBox^  txBox_targetDistant;
	private: System::Windows::Forms::TextBox^  txBox_AlphaBias;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  label12;
	private: System::Windows::Forms::Button^  Btn_UpDateSetting;
	private: System::IO::Ports::SerialPort^  serialPort_Radar_Angle;
#pragma endregion

#pragma region Windows Form Designer generated code
			 /// <summary>
			 /// 此為設計工具支援所需的方法 - 請勿使用程式碼編輯器修改
			 /// 這個方法的內容。
			 /// </summary>
			 void InitializeComponent(void)
			 {
				 this->components = (gcnew System::ComponentModel::Container());
				 System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea1 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
				 System::Windows::Forms::DataVisualization::Charting::Legend^  legend1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series3 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series4 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series5 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 this->serialPort_LiDAR = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
				 this->serialPort_Radar_Angle = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->tabPage2 = (gcnew System::Windows::Forms::TabPage());
				 this->groupBox7 = (gcnew System::Windows::Forms::GroupBox());
				 this->label12 = (gcnew System::Windows::Forms::Label());
				 this->label15 = (gcnew System::Windows::Forms::Label());
				 this->tBox_Partition = (gcnew System::Windows::Forms::TextBox());
				 this->groupBox6 = (gcnew System::Windows::Forms::GroupBox());
				 this->label6 = (gcnew System::Windows::Forms::Label());
				 this->label3 = (gcnew System::Windows::Forms::Label());
				 this->txBox_targetDistant = (gcnew System::Windows::Forms::TextBox());
				 this->txBox_AlphaBias = (gcnew System::Windows::Forms::TextBox());
				 this->label1 = (gcnew System::Windows::Forms::Label());
				 this->tabPage1 = (gcnew System::Windows::Forms::TabPage());
				 this->groupBox5 = (gcnew System::Windows::Forms::GroupBox());
				 this->label9 = (gcnew System::Windows::Forms::Label());
				 this->Btn_Radar_Connect = (gcnew System::Windows::Forms::Button());
				 this->cBox_Radar = (gcnew System::Windows::Forms::ComboBox());
				 this->tx_TBox_LAngle = (gcnew System::Windows::Forms::Label());
				 this->tx_TBox_RAngle = (gcnew System::Windows::Forms::Label());
				 this->label7 = (gcnew System::Windows::Forms::Label());
				 this->chart1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
				 this->groupBox4 = (gcnew System::Windows::Forms::GroupBox());
				 this->cBox_Record = (gcnew System::Windows::Forms::CheckBox());
				 this->Btn_Tbox_Close = (gcnew System::Windows::Forms::Button());
				 this->Tx_Radar_Mode = (gcnew System::Windows::Forms::Label());
				 this->cBox_TBox = (gcnew System::Windows::Forms::ComboBox());
				 this->Btn_Tbox_Connect = (gcnew System::Windows::Forms::Button());
				 this->label8 = (gcnew System::Windows::Forms::Label());
				 this->Tx_CarSpeed = (gcnew System::Windows::Forms::Label());
				 this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
				 this->tabControl2 = (gcnew System::Windows::Forms::TabControl());
				 this->tabPage3 = (gcnew System::Windows::Forms::TabPage());
				 this->tx_LRadarBias_Y = (gcnew System::Windows::Forms::Label());
				 this->tx_LRadarBias_X = (gcnew System::Windows::Forms::Label());
				 this->label5 = (gcnew System::Windows::Forms::Label());
				 this->label4 = (gcnew System::Windows::Forms::Label());
				 this->Btn_LeftBias = (gcnew System::Windows::Forms::Button());
				 this->tabPage4 = (gcnew System::Windows::Forms::TabPage());
				 this->ckBox_RadarR = (gcnew System::Windows::Forms::CheckBox());
				 this->tx_RRadarBias_X = (gcnew System::Windows::Forms::Label());
				 this->tx_RRadarBias_Y = (gcnew System::Windows::Forms::Label());
				 this->label10 = (gcnew System::Windows::Forms::Label());
				 this->label11 = (gcnew System::Windows::Forms::Label());
				 this->Btn_RightBias = (gcnew System::Windows::Forms::Button());
				 this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
				 this->Btn_Send_RadarAngle_Cmd = (gcnew System::Windows::Forms::Button());
				 this->Btn_RadarAngle_DisConnect = (gcnew System::Windows::Forms::Button());
				 this->lbBsdAngleT = (gcnew System::Windows::Forms::Label());
				 this->label2 = (gcnew System::Windows::Forms::Label());
				 this->Btn_RadarA_Connect = (gcnew System::Windows::Forms::Button());
				 this->cBox_Radar_Angle = (gcnew System::Windows::Forms::ComboBox());
				 this->Btn_Refresh_Combox = (gcnew System::Windows::Forms::Button());
				 this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
				 this->cBox_LiDAR = (gcnew System::Windows::Forms::ComboBox());
				 this->Btn_LiDAR_Connected = (gcnew System::Windows::Forms::Button());
				 this->Btn_LiDAR_DisConnect = (gcnew System::Windows::Forms::Button());
				 this->tabControl1 = (gcnew System::Windows::Forms::TabControl());
				 this->serialPort_Tbox = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->serialPort_Radar = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->Btn_UpDateSetting = (gcnew System::Windows::Forms::Button());
				 this->tabPage2->SuspendLayout();
				 this->groupBox7->SuspendLayout();
				 this->groupBox6->SuspendLayout();
				 this->tabPage1->SuspendLayout();
				 this->groupBox5->SuspendLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->BeginInit();
				 this->groupBox4->SuspendLayout();
				 this->groupBox3->SuspendLayout();
				 this->tabControl2->SuspendLayout();
				 this->tabPage3->SuspendLayout();
				 this->tabPage4->SuspendLayout();
				 this->groupBox2->SuspendLayout();
				 this->groupBox1->SuspendLayout();
				 this->tabControl1->SuspendLayout();
				 this->SuspendLayout();
				 // 
				 // serialPort_LiDAR
				 // 
				 this->serialPort_LiDAR->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_LiDAR_DataReceived);
				 // 
				 // timer1
				 // 
				 this->timer1->Tick += gcnew System::EventHandler(this, &MyForm::timer1_Tick);
				 // 
				 // serialPort_Radar_Angle
				 // 
				 this->serialPort_Radar_Angle->BaudRate = 19200;
				 this->serialPort_Radar_Angle->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_Radar_Angle_DataReceived);
				 // 
				 // tabPage2
				 // 
				 this->tabPage2->BackColor = System::Drawing::Color::White;
				 this->tabPage2->Controls->Add(this->Btn_UpDateSetting);
				 this->tabPage2->Controls->Add(this->groupBox7);
				 this->tabPage2->Controls->Add(this->groupBox6);
				 this->tabPage2->Location = System::Drawing::Point(4, 25);
				 this->tabPage2->Margin = System::Windows::Forms::Padding(4);
				 this->tabPage2->Name = L"tabPage2";
				 this->tabPage2->Padding = System::Windows::Forms::Padding(4);
				 this->tabPage2->Size = System::Drawing::Size(2552, 1321);
				 this->tabPage2->TabIndex = 1;
				 this->tabPage2->Text = L"設定";
				 // 
				 // groupBox7
				 // 
				 this->groupBox7->Controls->Add(this->label12);
				 this->groupBox7->Controls->Add(this->label15);
				 this->groupBox7->Controls->Add(this->tBox_Partition);
				 this->groupBox7->Location = System::Drawing::Point(4, 164);
				 this->groupBox7->Name = L"groupBox7";
				 this->groupBox7->Size = System::Drawing::Size(244, 85);
				 this->groupBox7->TabIndex = 16;
				 this->groupBox7->TabStop = false;
				 this->groupBox7->Text = L"LiDARSetting";
				 // 
				 // label12
				 // 
				 this->label12->AutoSize = true;
				 this->label12->Location = System::Drawing::Point(213, 50);
				 this->label12->Name = L"label12";
				 this->label12->Size = System::Drawing::Size(28, 15);
				 this->label12->TabIndex = 2;
				 this->label12->Text = L"(m)";
				 // 
				 // label15
				 // 
				 this->label15->AutoSize = true;
				 this->label15->Location = System::Drawing::Point(3, 51);
				 this->label15->Name = L"label15";
				 this->label15->Size = System::Drawing::Size(97, 15);
				 this->label15->TabIndex = 1;
				 this->label15->Text = L"Partition Value:";
				 // 
				 // tBox_Partition
				 // 
				 this->tBox_Partition->Location = System::Drawing::Point(106, 41);
				 this->tBox_Partition->Name = L"tBox_Partition";
				 this->tBox_Partition->Size = System::Drawing::Size(100, 25);
				 this->tBox_Partition->TabIndex = 0;
				 this->tBox_Partition->Text = L"1";
				 // 
				 // groupBox6
				 // 
				 this->groupBox6->Controls->Add(this->label6);
				 this->groupBox6->Controls->Add(this->label3);
				 this->groupBox6->Controls->Add(this->txBox_targetDistant);
				 this->groupBox6->Controls->Add(this->txBox_AlphaBias);
				 this->groupBox6->Controls->Add(this->label1);
				 this->groupBox6->Location = System::Drawing::Point(7, 23);
				 this->groupBox6->Name = L"groupBox6";
				 this->groupBox6->Size = System::Drawing::Size(216, 100);
				 this->groupBox6->TabIndex = 15;
				 this->groupBox6->TabStop = false;
				 this->groupBox6->Text = L"AngleRadarSetting";
				 // 
				 // label6
				 // 
				 this->label6->AutoSize = true;
				 this->label6->Location = System::Drawing::Point(176, 82);
				 this->label6->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label6->Name = L"label6";
				 this->label6->Size = System::Drawing::Size(28, 15);
				 this->label6->TabIndex = 14;
				 this->label6->Text = L"(m)";
				 // 
				 // label3
				 // 
				 this->label3->AutoSize = true;
				 this->label3->Location = System::Drawing::Point(7, 82);
				 this->label3->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label3->Name = L"label3";
				 this->label3->Size = System::Drawing::Size(87, 15);
				 this->label3->TabIndex = 13;
				 this->label3->Text = L"target Distant:";
				 // 
				 // txBox_targetDistant
				 // 
				 this->txBox_targetDistant->Location = System::Drawing::Point(107, 70);
				 this->txBox_targetDistant->Margin = System::Windows::Forms::Padding(4);
				 this->txBox_targetDistant->Name = L"txBox_targetDistant";
				 this->txBox_targetDistant->Size = System::Drawing::Size(60, 25);
				 this->txBox_targetDistant->TabIndex = 12;
				 this->txBox_targetDistant->Text = L"2.9";
				 // 
				 // txBox_AlphaBias
				 // 
				 this->txBox_AlphaBias->Location = System::Drawing::Point(107, 20);
				 this->txBox_AlphaBias->Margin = System::Windows::Forms::Padding(4);
				 this->txBox_AlphaBias->Name = L"txBox_AlphaBias";
				 this->txBox_AlphaBias->ReadOnly = true;
				 this->txBox_AlphaBias->Size = System::Drawing::Size(60, 25);
				 this->txBox_AlphaBias->TabIndex = 11;
				 this->txBox_AlphaBias->Text = L"-2.84";
				 // 
				 // label1
				 // 
				 this->label1->AutoSize = true;
				 this->label1->Location = System::Drawing::Point(7, 32);
				 this->label1->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label1->Name = L"label1";
				 this->label1->Size = System::Drawing::Size(73, 15);
				 this->label1->TabIndex = 10;
				 this->label1->Text = L"Alpha Bias:";
				 // 
				 // tabPage1
				 // 
				 this->tabPage1->Controls->Add(this->groupBox5);
				 this->tabPage1->Controls->Add(this->tx_TBox_LAngle);
				 this->tabPage1->Controls->Add(this->tx_TBox_RAngle);
				 this->tabPage1->Controls->Add(this->label7);
				 this->tabPage1->Controls->Add(this->chart1);
				 this->tabPage1->Controls->Add(this->groupBox4);
				 this->tabPage1->Controls->Add(this->groupBox3);
				 this->tabPage1->Controls->Add(this->groupBox2);
				 this->tabPage1->Controls->Add(this->Btn_Refresh_Combox);
				 this->tabPage1->Controls->Add(this->groupBox1);
				 this->tabPage1->Location = System::Drawing::Point(4, 25);
				 this->tabPage1->Margin = System::Windows::Forms::Padding(4);
				 this->tabPage1->Name = L"tabPage1";
				 this->tabPage1->Padding = System::Windows::Forms::Padding(4);
				 this->tabPage1->Size = System::Drawing::Size(2552, 1321);
				 this->tabPage1->TabIndex = 0;
				 this->tabPage1->Text = L"圖";
				 this->tabPage1->UseVisualStyleBackColor = true;
				 // 
				 // groupBox5
				 // 
				 this->groupBox5->Controls->Add(this->label9);
				 this->groupBox5->Controls->Add(this->Btn_Radar_Connect);
				 this->groupBox5->Controls->Add(this->cBox_Radar);
				 this->groupBox5->Location = System::Drawing::Point(12, 790);
				 this->groupBox5->Margin = System::Windows::Forms::Padding(4);
				 this->groupBox5->Name = L"groupBox5";
				 this->groupBox5->Padding = System::Windows::Forms::Padding(4);
				 this->groupBox5->Size = System::Drawing::Size(267, 105);
				 this->groupBox5->TabIndex = 12;
				 this->groupBox5->TabStop = false;
				 this->groupBox5->Text = L"Radar";
				 // 
				 // label9
				 // 
				 this->label9->AutoSize = true;
				 this->label9->Location = System::Drawing::Point(33, 82);
				 this->label9->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label9->Name = L"label9";
				 this->label9->Size = System::Drawing::Size(41, 15);
				 this->label9->TabIndex = 14;
				 this->label9->Text = L"label9";
				 // 
				 // Btn_Radar_Connect
				 // 
				 this->Btn_Radar_Connect->Location = System::Drawing::Point(161, 26);
				 this->Btn_Radar_Connect->Margin = System::Windows::Forms::Padding(4);
				 this->Btn_Radar_Connect->Name = L"Btn_Radar_Connect";
				 this->Btn_Radar_Connect->Size = System::Drawing::Size(100, 29);
				 this->Btn_Radar_Connect->TabIndex = 13;
				 this->Btn_Radar_Connect->Text = L"連接";
				 this->Btn_Radar_Connect->UseVisualStyleBackColor = true;
				 this->Btn_Radar_Connect->Click += gcnew System::EventHandler(this, &MyForm::Btn_Radar_Connect_Click);
				 // 
				 // cBox_Radar
				 // 
				 this->cBox_Radar->FormattingEnabled = true;
				 this->cBox_Radar->Location = System::Drawing::Point(8, 26);
				 this->cBox_Radar->Margin = System::Windows::Forms::Padding(4);
				 this->cBox_Radar->Name = L"cBox_Radar";
				 this->cBox_Radar->Size = System::Drawing::Size(132, 23);
				 this->cBox_Radar->TabIndex = 12;
				 // 
				 // tx_TBox_LAngle
				 // 
				 this->tx_TBox_LAngle->AutoSize = true;
				 this->tx_TBox_LAngle->Location = System::Drawing::Point(549, 739);
				 this->tx_TBox_LAngle->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->tx_TBox_LAngle->Name = L"tx_TBox_LAngle";
				 this->tx_TBox_LAngle->Size = System::Drawing::Size(48, 15);
				 this->tx_TBox_LAngle->TabIndex = 10;
				 this->tx_TBox_LAngle->Text = L"label12";
				 // 
				 // tx_TBox_RAngle
				 // 
				 this->tx_TBox_RAngle->AutoSize = true;
				 this->tx_TBox_RAngle->Location = System::Drawing::Point(321, 739);
				 this->tx_TBox_RAngle->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->tx_TBox_RAngle->Name = L"tx_TBox_RAngle";
				 this->tx_TBox_RAngle->Size = System::Drawing::Size(41, 15);
				 this->tx_TBox_RAngle->TabIndex = 9;
				 this->tx_TBox_RAngle->Text = L"label9";
				 // 
				 // label7
				 // 
				 this->label7->AutoSize = true;
				 this->label7->BackColor = System::Drawing::Color::White;
				 this->label7->Location = System::Drawing::Point(1669, 122);
				 this->label7->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label7->Name = L"label7";
				 this->label7->Size = System::Drawing::Size(58, 15);
				 this->label7->TabIndex = 8;
				 this->label7->Text = L"單位:cm";
				 // 
				 // chart1
				 // 
				 chartArea1->AxisX->Interval = 100;
				 chartArea1->AxisX->Maximum = 2000;
				 chartArea1->AxisX->Minimum = -2000;
				 chartArea1->AxisY->Interval = 100;
				 chartArea1->AxisY->Maximum = 2000;
				 chartArea1->AxisY->Minimum = 0;
				 chartArea1->Name = L"ChartArea1";
				 this->chart1->ChartAreas->Add(chartArea1);
				 legend1->Name = L"Legend1";
				 this->chart1->Legends->Add(legend1);
				 this->chart1->Location = System::Drawing::Point(276, 0);
				 this->chart1->Margin = System::Windows::Forms::Padding(4);
				 this->chart1->Name = L"chart1";
				 series1->ChartArea = L"ChartArea1";
				 series1->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series1->Color = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(255)),
					 static_cast<System::Int32>(static_cast<System::Byte>(128)));
				 series1->Legend = L"Legend1";
				 series1->Name = L"Series_LiDAR";
				 series2->ChartArea = L"ChartArea1";
				 series2->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series2->Color = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)),
					 static_cast<System::Int32>(static_cast<System::Byte>(192)));
				 series2->Legend = L"Legend1";
				 series2->MarkerColor = System::Drawing::Color::Blue;
				 series2->MarkerSize = 10;
				 series2->Name = L"Series_LiDAR_CLOSE";
				 series3->ChartArea = L"ChartArea1";
				 series3->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::FastPoint;
				 series3->Color = System::Drawing::Color::ForestGreen;
				 series3->Legend = L"Legend1";
				 series3->MarkerSize = 10;
				 series3->Name = L"Series_Radar_Angle";
				 series4->ChartArea = L"ChartArea1";
				 series4->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series4->Legend = L"Legend1";
				 series4->MarkerColor = System::Drawing::SystemColors::MenuHighlight;
				 series4->MarkerSize = 10;
				 series4->MarkerStyle = System::Windows::Forms::DataVisualization::Charting::MarkerStyle::Star4;
				 series4->Name = L"Series_TBox_LRadar";
				 series5->ChartArea = L"ChartArea1";
				 series5->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series5->Color = System::Drawing::Color::Black;
				 series5->Legend = L"Legend1";
				 series5->MarkerSize = 10;
				 series5->Name = L"Series_TBox_RRadar";
				 this->chart1->Series->Add(series1);
				 this->chart1->Series->Add(series2);
				 this->chart1->Series->Add(series3);
				 this->chart1->Series->Add(series4);
				 this->chart1->Series->Add(series5);
				 this->chart1->Size = System::Drawing::Size(1629, 750);
				 this->chart1->TabIndex = 8;
				 this->chart1->Text = L"圖";
				 // 
				 // groupBox4
				 // 
				 this->groupBox4->Controls->Add(this->cBox_Record);
				 this->groupBox4->Controls->Add(this->Btn_Tbox_Close);
				 this->groupBox4->Controls->Add(this->Tx_Radar_Mode);
				 this->groupBox4->Controls->Add(this->cBox_TBox);
				 this->groupBox4->Controls->Add(this->Btn_Tbox_Connect);
				 this->groupBox4->Controls->Add(this->label8);
				 this->groupBox4->Controls->Add(this->Tx_CarSpeed);
				 this->groupBox4->Location = System::Drawing::Point(25, 629);
				 this->groupBox4->Margin = System::Windows::Forms::Padding(4);
				 this->groupBox4->Name = L"groupBox4";
				 this->groupBox4->Padding = System::Windows::Forms::Padding(4);
				 this->groupBox4->Size = System::Drawing::Size(243, 140);
				 this->groupBox4->TabIndex = 7;
				 this->groupBox4->TabStop = false;
				 this->groupBox4->Text = L"TBox";
				 // 
				 // cBox_Record
				 // 
				 this->cBox_Record->AutoSize = true;
				 this->cBox_Record->Location = System::Drawing::Point(49, 0);
				 this->cBox_Record->Name = L"cBox_Record";
				 this->cBox_Record->Size = System::Drawing::Size(89, 19);
				 this->cBox_Record->TabIndex = 13;
				 this->cBox_Record->Text = L"繪圖模式";
				 this->cBox_Record->UseVisualStyleBackColor = true;
				 // 
				 // Btn_Tbox_Close
				 // 
				 this->Btn_Tbox_Close->Location = System::Drawing::Point(136, 74);
				 this->Btn_Tbox_Close->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
				 this->Btn_Tbox_Close->Name = L"Btn_Tbox_Close";
				 this->Btn_Tbox_Close->Size = System::Drawing::Size(100, 29);
				 this->Btn_Tbox_Close->TabIndex = 12;
				 this->Btn_Tbox_Close->Text = L"關閉";
				 this->Btn_Tbox_Close->UseVisualStyleBackColor = true;
				 this->Btn_Tbox_Close->Click += gcnew System::EventHandler(this, &MyForm::Btn_Tbox_Close_Click);
				 // 
				 // Tx_Radar_Mode
				 // 
				 this->Tx_Radar_Mode->AutoSize = true;
				 this->Tx_Radar_Mode->Location = System::Drawing::Point(8, 110);
				 this->Tx_Radar_Mode->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->Tx_Radar_Mode->Name = L"Tx_Radar_Mode";
				 this->Tx_Radar_Mode->Size = System::Drawing::Size(40, 15);
				 this->Tx_Radar_Mode->TabIndex = 11;
				 this->Tx_Radar_Mode->Text = L"Mode";
				 // 
				 // cBox_TBox
				 // 
				 this->cBox_TBox->FormattingEnabled = true;
				 this->cBox_TBox->Location = System::Drawing::Point(8, 26);
				 this->cBox_TBox->Margin = System::Windows::Forms::Padding(4);
				 this->cBox_TBox->Name = L"cBox_TBox";
				 this->cBox_TBox->Size = System::Drawing::Size(115, 23);
				 this->cBox_TBox->TabIndex = 7;
				 // 
				 // Btn_Tbox_Connect
				 // 
				 this->Btn_Tbox_Connect->Location = System::Drawing::Point(135, 26);
				 this->Btn_Tbox_Connect->Margin = System::Windows::Forms::Padding(4);
				 this->Btn_Tbox_Connect->Name = L"Btn_Tbox_Connect";
				 this->Btn_Tbox_Connect->Size = System::Drawing::Size(100, 29);
				 this->Btn_Tbox_Connect->TabIndex = 8;
				 this->Btn_Tbox_Connect->Text = L"連接";
				 this->Btn_Tbox_Connect->UseVisualStyleBackColor = true;
				 this->Btn_Tbox_Connect->Click += gcnew System::EventHandler(this, &MyForm::Btn_Tbox_Connect_Click);
				 // 
				 // label8
				 // 
				 this->label8->AutoSize = true;
				 this->label8->Location = System::Drawing::Point(8, 74);
				 this->label8->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label8->Name = L"label8";
				 this->label8->Size = System::Drawing::Size(71, 15);
				 this->label8->TabIndex = 9;
				 this->label8->Text = L"目前車速:";
				 // 
				 // Tx_CarSpeed
				 // 
				 this->Tx_CarSpeed->AutoSize = true;
				 this->Tx_CarSpeed->Location = System::Drawing::Point(109, 74);
				 this->Tx_CarSpeed->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->Tx_CarSpeed->Name = L"Tx_CarSpeed";
				 this->Tx_CarSpeed->Size = System::Drawing::Size(14, 15);
				 this->Tx_CarSpeed->TabIndex = 10;
				 this->Tx_CarSpeed->Text = L"0";
				 // 
				 // groupBox3
				 // 
				 this->groupBox3->Controls->Add(this->tabControl2);
				 this->groupBox3->Location = System::Drawing::Point(25, 464);
				 this->groupBox3->Margin = System::Windows::Forms::Padding(4);
				 this->groupBox3->Name = L"groupBox3";
				 this->groupBox3->Padding = System::Windows::Forms::Padding(4);
				 this->groupBox3->Size = System::Drawing::Size(243, 158);
				 this->groupBox3->TabIndex = 5;
				 this->groupBox3->TabStop = false;
				 this->groupBox3->Text = L"對位";
				 // 
				 // tabControl2
				 // 
				 this->tabControl2->Controls->Add(this->tabPage3);
				 this->tabControl2->Controls->Add(this->tabPage4);
				 this->tabControl2->Location = System::Drawing::Point(11, 26);
				 this->tabControl2->Margin = System::Windows::Forms::Padding(4);
				 this->tabControl2->Name = L"tabControl2";
				 this->tabControl2->SelectedIndex = 0;
				 this->tabControl2->Size = System::Drawing::Size(221, 126);
				 this->tabControl2->TabIndex = 0;
				 // 
				 // tabPage3
				 // 
				 this->tabPage3->Controls->Add(this->tx_LRadarBias_Y);
				 this->tabPage3->Controls->Add(this->tx_LRadarBias_X);
				 this->tabPage3->Controls->Add(this->label5);
				 this->tabPage3->Controls->Add(this->label4);
				 this->tabPage3->Controls->Add(this->Btn_LeftBias);
				 this->tabPage3->Location = System::Drawing::Point(4, 25);
				 this->tabPage3->Margin = System::Windows::Forms::Padding(4);
				 this->tabPage3->Name = L"tabPage3";
				 this->tabPage3->Padding = System::Windows::Forms::Padding(4);
				 this->tabPage3->Size = System::Drawing::Size(213, 97);
				 this->tabPage3->TabIndex = 0;
				 this->tabPage3->Text = L"左邊雷達";
				 this->tabPage3->UseVisualStyleBackColor = true;
				 // 
				 // tx_LRadarBias_Y
				 // 
				 this->tx_LRadarBias_Y->AutoSize = true;
				 this->tx_LRadarBias_Y->Location = System::Drawing::Point(123, 51);
				 this->tx_LRadarBias_Y->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->tx_LRadarBias_Y->Name = L"tx_LRadarBias_Y";
				 this->tx_LRadarBias_Y->Size = System::Drawing::Size(41, 15);
				 this->tx_LRadarBias_Y->TabIndex = 3;
				 this->tx_LRadarBias_Y->Text = L"label7";
				 // 
				 // tx_LRadarBias_X
				 // 
				 this->tx_LRadarBias_X->AutoSize = true;
				 this->tx_LRadarBias_X->Location = System::Drawing::Point(123, 19);
				 this->tx_LRadarBias_X->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->tx_LRadarBias_X->Name = L"tx_LRadarBias_X";
				 this->tx_LRadarBias_X->Size = System::Drawing::Size(41, 15);
				 this->tx_LRadarBias_X->TabIndex = 2;
				 this->tx_LRadarBias_X->Text = L"label6";
				 // 
				 // label5
				 // 
				 this->label5->AutoSize = true;
				 this->label5->Location = System::Drawing::Point(92, 52);
				 this->label5->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label5->Name = L"label5";
				 this->label5->Size = System::Drawing::Size(21, 15);
				 this->label5->TabIndex = 1;
				 this->label5->Text = L"Y:";
				 // 
				 // label4
				 // 
				 this->label4->AutoSize = true;
				 this->label4->Location = System::Drawing::Point(92, 20);
				 this->label4->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label4->Name = L"label4";
				 this->label4->Size = System::Drawing::Size(21, 15);
				 this->label4->TabIndex = 1;
				 this->label4->Text = L"X:";
				 // 
				 // Btn_LeftBias
				 // 
				 this->Btn_LeftBias->Location = System::Drawing::Point(4, 8);
				 this->Btn_LeftBias->Margin = System::Windows::Forms::Padding(4);
				 this->Btn_LeftBias->Name = L"Btn_LeftBias";
				 this->Btn_LeftBias->Size = System::Drawing::Size(77, 78);
				 this->Btn_LeftBias->TabIndex = 0;
				 this->Btn_LeftBias->Text = L"確定";
				 this->Btn_LeftBias->UseVisualStyleBackColor = true;
				 this->Btn_LeftBias->Click += gcnew System::EventHandler(this, &MyForm::Btn_LeftBias_Click);
				 // 
				 // tabPage4
				 // 
				 this->tabPage4->Controls->Add(this->ckBox_RadarR);
				 this->tabPage4->Controls->Add(this->tx_RRadarBias_X);
				 this->tabPage4->Controls->Add(this->tx_RRadarBias_Y);
				 this->tabPage4->Controls->Add(this->label10);
				 this->tabPage4->Controls->Add(this->label11);
				 this->tabPage4->Controls->Add(this->Btn_RightBias);
				 this->tabPage4->Location = System::Drawing::Point(4, 25);
				 this->tabPage4->Margin = System::Windows::Forms::Padding(4);
				 this->tabPage4->Name = L"tabPage4";
				 this->tabPage4->Padding = System::Windows::Forms::Padding(4);
				 this->tabPage4->Size = System::Drawing::Size(213, 97);
				 this->tabPage4->TabIndex = 1;
				 this->tabPage4->Text = L"右邊雷達";
				 this->tabPage4->UseVisualStyleBackColor = true;
				 // 
				 // ckBox_RadarR
				 // 
				 this->ckBox_RadarR->AutoSize = true;
				 this->ckBox_RadarR->Location = System::Drawing::Point(89, 69);
				 this->ckBox_RadarR->Margin = System::Windows::Forms::Padding(4);
				 this->ckBox_RadarR->Name = L"ckBox_RadarR";
				 this->ckBox_RadarR->Size = System::Drawing::Size(104, 19);
				 this->ckBox_RadarR->TabIndex = 9;
				 this->ckBox_RadarR->Text = L"雷達在右邊";
				 this->ckBox_RadarR->UseVisualStyleBackColor = true;
				 // 
				 // tx_RRadarBias_X
				 // 
				 this->tx_RRadarBias_X->AutoSize = true;
				 this->tx_RRadarBias_X->Location = System::Drawing::Point(121, 20);
				 this->tx_RRadarBias_X->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->tx_RRadarBias_X->Name = L"tx_RRadarBias_X";
				 this->tx_RRadarBias_X->Size = System::Drawing::Size(41, 15);
				 this->tx_RRadarBias_X->TabIndex = 8;
				 this->tx_RRadarBias_X->Text = L"label8";
				 // 
				 // tx_RRadarBias_Y
				 // 
				 this->tx_RRadarBias_Y->AutoSize = true;
				 this->tx_RRadarBias_Y->Location = System::Drawing::Point(121, 51);
				 this->tx_RRadarBias_Y->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->tx_RRadarBias_Y->Name = L"tx_RRadarBias_Y";
				 this->tx_RRadarBias_Y->Size = System::Drawing::Size(41, 15);
				 this->tx_RRadarBias_Y->TabIndex = 7;
				 this->tx_RRadarBias_Y->Text = L"label9";
				 // 
				 // label10
				 // 
				 this->label10->AutoSize = true;
				 this->label10->Location = System::Drawing::Point(92, 51);
				 this->label10->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label10->Name = L"label10";
				 this->label10->Size = System::Drawing::Size(21, 15);
				 this->label10->TabIndex = 5;
				 this->label10->Text = L"Y:";
				 // 
				 // label11
				 // 
				 this->label11->AutoSize = true;
				 this->label11->Location = System::Drawing::Point(92, 20);
				 this->label11->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label11->Name = L"label11";
				 this->label11->Size = System::Drawing::Size(21, 15);
				 this->label11->TabIndex = 6;
				 this->label11->Text = L"X:";
				 // 
				 // Btn_RightBias
				 // 
				 this->Btn_RightBias->Location = System::Drawing::Point(4, 8);
				 this->Btn_RightBias->Margin = System::Windows::Forms::Padding(4);
				 this->Btn_RightBias->Name = L"Btn_RightBias";
				 this->Btn_RightBias->Size = System::Drawing::Size(77, 78);
				 this->Btn_RightBias->TabIndex = 4;
				 this->Btn_RightBias->Text = L"確定";
				 this->Btn_RightBias->UseVisualStyleBackColor = true;
				 this->Btn_RightBias->Click += gcnew System::EventHandler(this, &MyForm::Btn_RightBias_Click);
				 // 
				 // groupBox2
				 // 
				 this->groupBox2->Controls->Add(this->Btn_Send_RadarAngle_Cmd);
				 this->groupBox2->Controls->Add(this->Btn_RadarAngle_DisConnect);
				 this->groupBox2->Controls->Add(this->lbBsdAngleT);
				 this->groupBox2->Controls->Add(this->label2);
				 this->groupBox2->Controls->Add(this->Btn_RadarA_Connect);
				 this->groupBox2->Controls->Add(this->cBox_Radar_Angle);
				 this->groupBox2->Location = System::Drawing::Point(25, 208);
				 this->groupBox2->Margin = System::Windows::Forms::Padding(4);
				 this->groupBox2->Name = L"groupBox2";
				 this->groupBox2->Padding = System::Windows::Forms::Padding(4);
				 this->groupBox2->Size = System::Drawing::Size(243, 238);
				 this->groupBox2->TabIndex = 3;
				 this->groupBox2->TabStop = false;
				 this->groupBox2->Text = L"Radar_Angle";
				 // 
				 // Btn_Send_RadarAngle_Cmd
				 // 
				 this->Btn_Send_RadarAngle_Cmd->Location = System::Drawing::Point(11, 78);
				 this->Btn_Send_RadarAngle_Cmd->Margin = System::Windows::Forms::Padding(4);
				 this->Btn_Send_RadarAngle_Cmd->Name = L"Btn_Send_RadarAngle_Cmd";
				 this->Btn_Send_RadarAngle_Cmd->Size = System::Drawing::Size(100, 29);
				 this->Btn_Send_RadarAngle_Cmd->TabIndex = 10;
				 this->Btn_Send_RadarAngle_Cmd->Text = L"Send BSD";
				 this->Btn_Send_RadarAngle_Cmd->UseVisualStyleBackColor = true;
				 this->Btn_Send_RadarAngle_Cmd->Click += gcnew System::EventHandler(this, &MyForm::Btn_Send_RadarAngle_Cmd_Click);
				 // 
				 // Btn_RadarAngle_DisConnect
				 // 
				 this->Btn_RadarAngle_DisConnect->Location = System::Drawing::Point(132, 78);
				 this->Btn_RadarAngle_DisConnect->Margin = System::Windows::Forms::Padding(4);
				 this->Btn_RadarAngle_DisConnect->Name = L"Btn_RadarAngle_DisConnect";
				 this->Btn_RadarAngle_DisConnect->Size = System::Drawing::Size(100, 29);
				 this->Btn_RadarAngle_DisConnect->TabIndex = 8;
				 this->Btn_RadarAngle_DisConnect->Text = L"關閉";
				 this->Btn_RadarAngle_DisConnect->UseVisualStyleBackColor = true;
				 this->Btn_RadarAngle_DisConnect->Click += gcnew System::EventHandler(this, &MyForm::Btn_RadarAngle_DisConnect_Click);
				 // 
				 // lbBsdAngleT
				 // 
				 this->lbBsdAngleT->AutoSize = true;
				 this->lbBsdAngleT->Location = System::Drawing::Point(80, 208);
				 this->lbBsdAngleT->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->lbBsdAngleT->Name = L"lbBsdAngleT";
				 this->lbBsdAngleT->Size = System::Drawing::Size(41, 15);
				 this->lbBsdAngleT->TabIndex = 5;
				 this->lbBsdAngleT->Text = L"label3";
				 // 
				 // label2
				 // 
				 this->label2->AutoSize = true;
				 this->label2->Location = System::Drawing::Point(11, 208);
				 this->label2->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label2->Name = L"label2";
				 this->label2->Size = System::Drawing::Size(45, 15);
				 this->label2->TabIndex = 4;
				 this->label2->Text = L"Angle:";
				 // 
				 // Btn_RadarA_Connect
				 // 
				 this->Btn_RadarA_Connect->Location = System::Drawing::Point(132, 26);
				 this->Btn_RadarA_Connect->Margin = System::Windows::Forms::Padding(4);
				 this->Btn_RadarA_Connect->Name = L"Btn_RadarA_Connect";
				 this->Btn_RadarA_Connect->Size = System::Drawing::Size(100, 29);
				 this->Btn_RadarA_Connect->TabIndex = 1;
				 this->Btn_RadarA_Connect->Text = L"連接";
				 this->Btn_RadarA_Connect->UseVisualStyleBackColor = true;
				 this->Btn_RadarA_Connect->Click += gcnew System::EventHandler(this, &MyForm::Btn_RadarA_Connect_Click);
				 // 
				 // cBox_Radar_Angle
				 // 
				 this->cBox_Radar_Angle->FormattingEnabled = true;
				 this->cBox_Radar_Angle->Location = System::Drawing::Point(8, 26);
				 this->cBox_Radar_Angle->Margin = System::Windows::Forms::Padding(4);
				 this->cBox_Radar_Angle->Name = L"cBox_Radar_Angle";
				 this->cBox_Radar_Angle->Size = System::Drawing::Size(115, 23);
				 this->cBox_Radar_Angle->TabIndex = 0;
				 // 
				 // Btn_Refresh_Combox
				 // 
				 this->Btn_Refresh_Combox->Location = System::Drawing::Point(12, 11);
				 this->Btn_Refresh_Combox->Margin = System::Windows::Forms::Padding(4);
				 this->Btn_Refresh_Combox->Name = L"Btn_Refresh_Combox";
				 this->Btn_Refresh_Combox->Size = System::Drawing::Size(100, 29);
				 this->Btn_Refresh_Combox->TabIndex = 2;
				 this->Btn_Refresh_Combox->Text = L"更新列表";
				 this->Btn_Refresh_Combox->UseVisualStyleBackColor = true;
				 this->Btn_Refresh_Combox->Click += gcnew System::EventHandler(this, &MyForm::Btn_Refresh_Combox_Click);
				 // 
				 // groupBox1
				 // 
				 this->groupBox1->Controls->Add(this->cBox_LiDAR);
				 this->groupBox1->Controls->Add(this->Btn_LiDAR_Connected);
				 this->groupBox1->Controls->Add(this->Btn_LiDAR_DisConnect);
				 this->groupBox1->Location = System::Drawing::Point(25, 61);
				 this->groupBox1->Margin = System::Windows::Forms::Padding(4);
				 this->groupBox1->Name = L"groupBox1";
				 this->groupBox1->Padding = System::Windows::Forms::Padding(4);
				 this->groupBox1->Size = System::Drawing::Size(243, 125);
				 this->groupBox1->TabIndex = 1;
				 this->groupBox1->TabStop = false;
				 this->groupBox1->Text = L"LiDAR";
				 // 
				 // cBox_LiDAR
				 // 
				 this->cBox_LiDAR->FormattingEnabled = true;
				 this->cBox_LiDAR->Location = System::Drawing::Point(11, 51);
				 this->cBox_LiDAR->Margin = System::Windows::Forms::Padding(4);
				 this->cBox_LiDAR->Name = L"cBox_LiDAR";
				 this->cBox_LiDAR->Size = System::Drawing::Size(115, 23);
				 this->cBox_LiDAR->TabIndex = 0;
				 // 
				 // Btn_LiDAR_Connected
				 // 
				 this->Btn_LiDAR_Connected->Location = System::Drawing::Point(135, 26);
				 this->Btn_LiDAR_Connected->Margin = System::Windows::Forms::Padding(4);
				 this->Btn_LiDAR_Connected->Name = L"Btn_LiDAR_Connected";
				 this->Btn_LiDAR_Connected->Size = System::Drawing::Size(100, 29);
				 this->Btn_LiDAR_Connected->TabIndex = 5;
				 this->Btn_LiDAR_Connected->Text = L"連接";
				 this->Btn_LiDAR_Connected->UseVisualStyleBackColor = true;
				 this->Btn_LiDAR_Connected->Click += gcnew System::EventHandler(this, &MyForm::Btn_LiDAR_Connected_Click);
				 // 
				 // Btn_LiDAR_DisConnect
				 // 
				 this->Btn_LiDAR_DisConnect->Location = System::Drawing::Point(135, 76);
				 this->Btn_LiDAR_DisConnect->Margin = System::Windows::Forms::Padding(4);
				 this->Btn_LiDAR_DisConnect->Name = L"Btn_LiDAR_DisConnect";
				 this->Btn_LiDAR_DisConnect->Size = System::Drawing::Size(100, 29);
				 this->Btn_LiDAR_DisConnect->TabIndex = 6;
				 this->Btn_LiDAR_DisConnect->Text = L"關閉";
				 this->Btn_LiDAR_DisConnect->UseVisualStyleBackColor = true;
				 this->Btn_LiDAR_DisConnect->Click += gcnew System::EventHandler(this, &MyForm::Btn_LiDAR_DisConnect_Click);
				 // 
				 // tabControl1
				 // 
				 this->tabControl1->Controls->Add(this->tabPage1);
				 this->tabControl1->Controls->Add(this->tabPage2);
				 this->tabControl1->Location = System::Drawing::Point(3, 15);
				 this->tabControl1->Margin = System::Windows::Forms::Padding(4);
				 this->tabControl1->Name = L"tabControl1";
				 this->tabControl1->SelectedIndex = 0;
				 this->tabControl1->Size = System::Drawing::Size(2560, 1350);
				 this->tabControl1->TabIndex = 0;
				 // 
				 // serialPort_Tbox
				 // 
				 this->serialPort_Tbox->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_Tbox_DataReceived);
				 // 
				 // serialPort_Radar
				 // 
				 this->serialPort_Radar->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_Radar_DataReceived);
				 // 
				 // Btn_UpDateSetting
				 // 
				 this->Btn_UpDateSetting->Location = System::Drawing::Point(60, 319);
				 this->Btn_UpDateSetting->Name = L"Btn_UpDateSetting";
				 this->Btn_UpDateSetting->Size = System::Drawing::Size(96, 39);
				 this->Btn_UpDateSetting->TabIndex = 17;
				 this->Btn_UpDateSetting->Text = L"更新設定值";
				 this->Btn_UpDateSetting->UseVisualStyleBackColor = true;
				 this->Btn_UpDateSetting->Click += gcnew System::EventHandler(this, &MyForm::Btn_UpDateSetting_Click);
				 // 
				 // MyForm
				 // 
				 this->AutoScaleDimensions = System::Drawing::SizeF(8, 15);
				 this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
				 this->AutoSizeMode = System::Windows::Forms::AutoSizeMode::GrowAndShrink;
				 this->ClientSize = System::Drawing::Size(1924, 1055);
				 this->Controls->Add(this->tabControl1);
				 this->Margin = System::Windows::Forms::Padding(4);
				 this->Name = L"MyForm";
				 this->Text = L"MyForm";
				 this->WindowState = System::Windows::Forms::FormWindowState::Maximized;
				 this->tabPage2->ResumeLayout(false);
				 this->groupBox7->ResumeLayout(false);
				 this->groupBox7->PerformLayout();
				 this->groupBox6->ResumeLayout(false);
				 this->groupBox6->PerformLayout();
				 this->tabPage1->ResumeLayout(false);
				 this->tabPage1->PerformLayout();
				 this->groupBox5->ResumeLayout(false);
				 this->groupBox5->PerformLayout();
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->EndInit();
				 this->groupBox4->ResumeLayout(false);
				 this->groupBox4->PerformLayout();
				 this->groupBox3->ResumeLayout(false);
				 this->tabControl2->ResumeLayout(false);
				 this->tabPage3->ResumeLayout(false);
				 this->tabPage3->PerformLayout();
				 this->tabPage4->ResumeLayout(false);
				 this->tabPage4->PerformLayout();
				 this->groupBox2->ResumeLayout(false);
				 this->groupBox2->PerformLayout();
				 this->groupBox1->ResumeLayout(false);
				 this->tabControl1->ResumeLayout(false);
				 this->ResumeLayout(false);

			 }
#pragma endregion
#pragma region DataReceive
	private: System::Void serialPort_Radar_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		uint Radar_buff_size = 12;
		byte bChecksum = 0;
		if (serialPort_Radar->BytesToRead < Radar_buff_size)
			return;
		cli::array<System::Byte>^ Radar_buff = gcnew cli::array<Byte>(Radar_buff_size);
		serialPort_Radar->Read(Radar_buff, 0, Radar_buff_size);

		if (Radar_buff[0] == 0x54)
		{
			for (uint i = 0; i < Radar_buff_size - 1; i++)
				bChecksum += Radar_buff[i];
			if (bChecksum == Radar_buff[11])
			{
				RadarData.ALert = Radar_buff[1];
				RadarData.Mode = Radar_buff[3];
				RadarData.Range = Radar_buff[8];
				RadarData.Speed = Radar_buff[9] - 127;
				RadarData.Angle = Radar_buff[10] - 127;
			}
		}
	}
	private: System::Void serialPort_LiDAR_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		cli::array<System::Byte>^ LiDAR_SerialPortData = gcnew cli::array<Byte>(10000);
		int ReadSize = serialPort_LiDAR->Read(LiDAR_SerialPortData, 0, 10000);

		for (int i = 0; i < ReadSize; i++)
		{
			if ((LiDAR_SerialPortData[i] == 0x02) && (LiDAR_SerialPortData[i + 1] == 0x80) && (LiDAR_SerialPortData[i + 4] == 0xB0) && (f_getHeader == false))
			{

				for (int j = i; j < ReadSize; j++)
				{
					LiDAR_Data[counter] = LiDAR_SerialPortData[j];
					counter++;
					if ((counter + 1) == 734)
						break;
				}
				f_getHeader = true;
				break;
			}
			if (f_getHeader)
			{
				if ((counter + 1) == 734)
					break;
				LiDAR_Data[counter] = LiDAR_SerialPortData[i];
				counter++;
			}
		}
		int Data[361];


		if ((counter + 1) == 734)
		{
			for (uint i = 0; i < 361; i++)
			{
				Data[i] = LiDAR_Data[2 * i + 7] + (LiDAR_Data[8 + i * 2] & 0x1F) * 256;
			}
			for (uint i = 0; i < 361; i++)
			{
				LIDAR_X_cooridate[i] = (Data[i]) * cos((0.5 * i) * (M_PI / 180));
				LIDAR_Y_cooridate[i] = (Data[i]) * sin((0.5 * i) * (M_PI / 180));
			}
			f_getLiDARData = true;
			f_getHeader = false;
			counter = 0;
		}
	}
	private: System::Void serialPort_Tbox_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		cli::array<System::Byte>^bTboxData = gcnew cli::array<Byte>(format);

		serialPort_Tbox->Read(bTboxData, 0, 1);
		if (bTboxData[0] == 0x80)
		{
			byte Checksum = 0;
			serialPort_Tbox->Read(bTboxData, 1, format - 1);
			for (uint i = 0; i < format - 1; i++)
			{
				Checksum += bTboxData[i];
			}
			if (Checksum == bTboxData[format - 1])
			{
				TBox.currentSpeed = bTboxData[9];
				TBox.L_RADAR_Mode = bTboxData[14];
				TBox.L_RADAR_ALert = bTboxData[15];
				TBox.L_RADAR_Range = bTboxData[16];
				TBox.L_RADAR_Speed = bTboxData[17] - 127;//- 127;
				TBox.L_RADAR_Angle = bTboxData[18] - 127;// -127;

				TBox.R_RADAR_Mode = bTboxData[19];
				TBox.R_RADAR_ALert = bTboxData[20];
				TBox.R_RADAR_Range = bTboxData[21];
				TBox.R_RADAR_Speed = bTboxData[22] - 127;//- 127;
				TBox.R_RADAR_Angle = bTboxData[23] - 127;// -127;
			}

		}
		if (serialPort_Tbox->BytesToRead >= format * 2)
		{
			serialPort_Tbox->DiscardInBuffer();
		}
	}
	private: System::Void serialPort_Radar_Angle_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		cli::array<System::Byte>^ bufferBsd = gcnew cli::array<Byte>(12);
		byte bChecksum = 0;
		if (serialPort_Radar_Angle->BytesToRead < 12)
			return;
		if ((byte)serialPort_Radar_Angle->ReadByte() == 0x54)
		{
			bufferBsd[0] = 0x54;
			serialPort_Radar_Angle->Read(bufferBsd, 1, 11);
			for (int i = 0; i < 11; i++) // checksum store in end of Frame 
			{
				bChecksum += bufferBsd[i];
			}
			if (bChecksum != bufferBsd[11])
				return;
			if (bufferBsd[1] == 0x64)//判斷是不是回傳資料
			{
				byte check_send = bufferBsd[9];
				return;
			}
			//判斷Command有沒有送
			if (bufferBsd[2] != 0xA1)
			{
				//bsd_Messige("Not valid BSD data!\r\n", "error");
				SetLabelText("Command沒送");
				//Loading_BSD = false;
				return;
			}
			//判斷有無目標
			if ((bufferBsd[1] != 1) || (bufferBsd[4] != 0xff) || (bufferBsd[5] != 0xff) || (bufferBsd[6] != 0xff))
			{
				SetLabelText("沒有目標");
				//bsd_Messige("No goal!\r\n", "error");

				return;
			}
			bsdAngle = ((bufferBsd[9] + bufferBsd[10] * 256) - 10000) / 100.0;
			AngleRadar_Point = Pt(targetDistant*Math::Cos(bsdAngle*M_PI / 180.f), targetDistant*Math::Sin(bsdAngle*M_PI / 180.f));

		}
		if (serialPort_Radar_Angle->BytesToRead >= 24)
		{
			serialPort_Radar_Angle->DiscardInBuffer();// To Flush the BSD Data  
		}

	}
	private:Pt CoordinateRotation(double degree, Pt P)
	{
		Pt Ans;
		Ans.x = cos(degree*M_PI / 180)*P.x + sin(degree*M_PI / 180)*P.y;
		Ans.y = -sin(degree*M_PI / 180)*P.x + cos(degree*M_PI / 180)*P.y;
		return Ans;
	}
	private:Pt R_Radar2LiDAR(Pt P)
	{
		Pt Ans;
		Pt Rotation = CoordinateRotation(-125, P);
		Ans.x = Rotation.x + right_Radar_bias.x;
		Ans.y = Rotation.y + right_Radar_bias.y;
		return Ans;
	}
	private:Pt L_Radar2LiDAR(Pt P)
	{
		Pt Ans;
		Pt Rotationtmp = CoordinateRotation(-35.0f, P);
		Ans.x = Rotationtmp.y + left_Radar_bias.x;
		Ans.y = Rotationtmp.x + left_Radar_bias.y;
		return Ans;
	}

#pragma endregion
	private:System::String^ getRadarMode(int index)
	{
		switch (index)
		{
		case 0x01:
			return "BSD Mode";

			break;
		case 0x02:
			return  "RCTA Mode";
			break;
		case 0x03:
			return  "DOW Mode";
			break;
		}
	}
	public:void ShowImage(System::Windows::Forms::PictureBox^ PBox, cv::Mat Image)
	{

		Mat image_Temp;
		switch (Image.type())
		{
		case CV_8UC3:
			image_Temp = Image;
			break;
		case CV_8UC1:
			cvtColor(Image, image_Temp, CV_GRAY2RGB);
			break;
		default:
			break;
		}
		System::IntPtr ptr(image_Temp.ptr());
		System::Drawing::Graphics^ graphics = PBox->CreateGraphics();
		System::Drawing::Bitmap^ b = gcnew System::Drawing::Bitmap(image_Temp.cols, image_Temp.rows, image_Temp.step, System::Drawing::Imaging::PixelFormat::Format24bppRgb, ptr);
		TextureBrush ^Brush = gcnew TextureBrush(b);

		System::Drawing::RectangleF rect(0, 0, PBox->Width, PBox->Height);
		graphics->FillRectangle(Brush, rect);
		delete Brush;
		delete graphics;
	}
	private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
		chart1->Series["Series_LiDAR"]->Points->Clear();
		chart1->Series["Series_LiDAR_CLOSE"]->Points->Clear();
		chart1->Series["Series_Radar_Angle"]->Points->Clear();
		if (!cBox_Record->Checked)
		{
			chart1->Series["Series_TBox_RRadar"]->Points->Clear();
			chart1->Series["Series_TBox_LRadar"]->Points->Clear();
		}


#pragma region 光達
		if (f_getLiDARData)
		{
			LIDAR_cooridate.resize(361);
			for (uint i = 0; i < 361; i++)
			{
				LIDAR_cooridate[i] = Pt(LIDAR_X_cooridate[i], LIDAR_Y_cooridate[i]);
				chart1->Series["Series_LiDAR"]->Points->AddXY(LIDAR_X_cooridate[i], LIDAR_Y_cooridate[i]);
			}
			cv::vector<int>Lab;
			int NoObj = partition(LIDAR_cooridate, Lab);
			vector<Pt> Pt_NewCluster = CaculateAveragePoint(LIDAR_cooridate, Lab, NoObj);
			if (Pt_OldCluster.size() == 0)Pt_OldCluster.resize(Pt_NewCluster.size());
			time_t t2 = clock();
			float time = (float)(t2 - t1) / CLK_TCK;
			FindClosePoint(Pt_NewCluster, Pt_OldCluster,time);
		
			t1 = t2;
			Pt_OldCluster = Pt_NewCluster;
			for (uint i = 0; i < Pt_NewCluster.size(); i++)
			{
				chart1->Series["Series_LiDAR_CLOSE"]->Points->AddXY(Pt_NewCluster[i].x, Pt_NewCluster[i].y);
				chart1->Series["Series_LiDAR_CLOSE"]->Label = "(" + Math::Round(Pt_NewCluster[i].x, 2).ToString() + " , " + Math::Round(Pt_NewCluster[i].y, 2).ToString() + " , " + Math::Round(Pt_NewCluster[i].velcity, 2).ToString() + ")";
			}
			
		}
#pragma endregion


#pragma region 純角度的雷達
		if (serialPort_Radar_Angle->IsOpen)
		{
			lbBsdAngleT->Text = Math::Round(bsdAngle, 2).ToString();
			Pt Radar_Angle_Point;
			if (ckBox_RadarR->Checked)
			{
				Pt Rotation = CoordinateRotation(-125.0f, AngleRadar_Point);
				Radar_Angle_Point.x = Rotation.x + right_Radar_bias.x;
				Radar_Angle_Point.y = Rotation.y + right_Radar_bias.y;

			}
			else
			{
				Pt Rotationtmp = CoordinateRotation(-35.0f, AngleRadar_Point);
				Radar_Angle_Point.x = Rotationtmp.y + left_Radar_bias.x;
				Radar_Angle_Point.y = Rotationtmp.x + left_Radar_bias.y;
			}
			chart1->Series["Series_Radar_Angle"]->Points->AddXY(Radar_Angle_Point.x, Radar_Angle_Point.y);
		}
#pragma endregion

#pragma region TBox
		if (serialPort_Tbox->IsOpen)
		{
			if (TBox.R_RADAR_ALert)
			{
				fstream fp;
				fp.open(RRadarFileName, ios::out | ios::app);
				fp << LiDAR_tmpPt.x << " " << LiDAR_tmpPt.y << " " << TBox.R_RADAR_Range << " " << TBox.R_RADAR_Angle << " " << TBox.R_RADAR_Speed << endl;
				fp.close();

				Pt R_RadarPtAtLiDAR = R_Radar2LiDAR(Pt(100 * TBox.R_RADAR_Range*Math::Cos(TBox.R_RADAR_Angle*M_PI / 180.f), 100 * TBox.R_RADAR_Range*Math::Sin(TBox.R_RADAR_Angle*M_PI / 180.f)));
				tx_TBox_RAngle->ForeColor = Color::Red;
				tx_TBox_RAngle->Text = "R Range: " + TBox.R_RADAR_Range.ToString() + " R Angle: " + TBox.R_RADAR_Angle.ToString();
				chart1->Series["Series_TBox_RRadar"]->Points->AddXY(R_RadarPtAtLiDAR.x, R_RadarPtAtLiDAR.y);

			}
			else
				tx_TBox_RAngle->ForeColor = Color::Blue;

			if (TBox.L_RADAR_ALert)
			{
				fstream fp;
				fp.open(LRadarFileName, ios::out | ios::app);
				fp << LiDAR_tmpPt.x << " " << LiDAR_tmpPt.y << " " << TBox.L_RADAR_Range << " " << TBox.L_RADAR_Angle << " " << TBox.L_RADAR_Speed << endl;
				fp.close();
				tx_TBox_LAngle->ForeColor = Color::Red;
				tx_TBox_LAngle->Text = "L Range: " + TBox.L_RADAR_Range.ToString() + " L Angle: " + TBox.L_RADAR_Angle.ToString();
				Pt L_RadarPtAtLiDAR = L_Radar2LiDAR(Pt(100 * TBox.L_RADAR_Range*Math::Cos(TBox.L_RADAR_Angle*M_PI / 180.f), 100 * TBox.L_RADAR_Range*Math::Sin(TBox.L_RADAR_Angle*M_PI / 180.f)));
				chart1->Series["Series_TBox_LRadar"]->Points->AddXY(L_RadarPtAtLiDAR.x, L_RadarPtAtLiDAR.y);
			}
			else
				tx_TBox_LAngle->ForeColor = Color::Blue;
			Tx_CarSpeed->Text = TBox.currentSpeed.ToString();
			Tx_Radar_Mode->Text = "L:" + getRadarMode(TBox.L_RADAR_Mode) + "  R:" + getRadarMode(TBox.R_RADAR_Mode);
		}
#pragma endregion

#pragma region DB9雷達
		if (serialPort_Radar->IsOpen)
		{
			if (RadarData.ALert > 0)
			{
				fstream fptemp;
				char RadarFileName[100];
				int Date = System::DateTime::Now.Day * 10000 + System::DateTime::Now.Hour * 100 + System::DateTime::Now.Minute;
				sprintf(RadarFileName, ".\\Data\\RadarData%d.txt", Date);
				fptemp.open(RadarFileName, ios::out | ios::app);
				fptemp << RadarData.Range << "\t" << RadarData.Angle << "\t" << RadarData.Speed << endl;
				fptemp.close();
				label9->Text = RadarData.Range.ToString() + " " + RadarData.Angle.ToString();
			}
		}
#pragma endregion	
		chart1->Refresh();
	}
			 delegate void SetLabel(System::String^ str);
	private:void SetLabelText(System::String^ str)
	{
		if (this->lbBsdAngleT->InvokeRequired)
		{
			SetLabel ^d = gcnew SetLabel(this, &MyForm::SetLabelText);
			this->Invoke(d, gcnew cli::array<Object^> { str });
		}
		else
		{
			this->lbBsdAngleT->Text = str;
			this->lbBsdAngleT->Refresh();
		}
	}
#pragma region Btn事件
	private: System::Void Btn_Radar_Connect_Click(System::Object^  sender, System::EventArgs^  e) {
		serialPort_Radar->PortName = cBox_Radar->Text;
		serialPort_Radar->BaudRate = 460800;
		serialPort_Radar->DataBits = 8;
		serialPort_Radar->StopBits = StopBits::One;
		serialPort_Radar->Parity = Parity::None;
		serialPort_Radar->Open();
	}
	private: System::Void Btn_Send_RadarAngle_Cmd_Click(System::Object^  sender, System::EventArgs^  e) {
		cli::array<byte>^ cmd = gcnew cli::array<System::Byte>{ 0x80, 0x64, 0xA1, 0x1F, 0x7C, 0x00, 0x00, 0xAB, 0xFD, 0x01, 0x00, 0xC9};
		serialPort_Radar_Angle->Write(cmd, 0, 12);
	}
	private: System::Void Btn_Tbox_Connect_Click(System::Object^  sender, System::EventArgs^  e) {
		if (serialPort_Tbox->IsOpen)
		{

			serialPort_Tbox->Close();
			Sleep(10);
		}
		serialPort_Tbox->PortName = cBox_TBox->Text;
		serialPort_Tbox->BaudRate = 115200;
		serialPort_Tbox->DataBits = 8;
		serialPort_Tbox->StopBits = StopBits::One;
		serialPort_Tbox->Parity = Parity::None;
		serialPort_Tbox->Open();

	}
	private: System::Void Btn_Tbox_Close_Click(System::Object^  sender, System::EventArgs^  e) {
		serialPort_Tbox->Close();
	}
	private:void ComPortRefresh(void)
	{
		cBox_LiDAR->Items->Clear();
		cBox_Radar_Angle->Items->Clear();
		cBox_TBox->Items->Clear();
		cBox_Radar->Items->Clear();
		cli::array<System::String^>^ Port = SerialPort::GetPortNames();
		cBox_LiDAR->Items->AddRange(Port);
		cBox_Radar_Angle->Items->AddRange(Port);
		cBox_TBox->Items->AddRange(Port);
		cBox_Radar->Items->AddRange(Port);
	}
	private: System::Void Btn_LiDAR_Connected_Click(System::Object^  sender, System::EventArgs^  e) {
		if (serialPort_LiDAR->IsOpen)serialPort_LiDAR->Close();

		serialPort_LiDAR->PortName = cBox_LiDAR->Text;
		serialPort_LiDAR->Encoding = System::Text::Encoding::GetEncoding(28591);
		serialPort_LiDAR->BaudRate = 9600;
		serialPort_LiDAR->DataBits = 8;
		serialPort_LiDAR->StopBits = StopBits::One;
		serialPort_LiDAR->Parity = Parity::None;
		serialPort_LiDAR->Open();

		cli::array<System::Byte>^ LMS_Angular_range_change_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x05, 0x00, 0x3B, 0xB4, 0x00, 0x32, 0x00, 0x3B, 0x1F };//更改LMS經度0.5度
		cli::array<System::Byte>^ continuous_LMS_data_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x24, 0x34, 0x08 };//更改成連續指令緩區
		cli::array<System::Byte>^ LMS_baundrate_500k_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x48, 0x58, 0x08 };//更改包率

		if (serialPort_LiDAR->IsOpen)
		{
			serialPort_LiDAR->Write(LMS_baundrate_500k_manage, 0, 8);
			Sleep(500);
			serialPort_LiDAR->Close();
			serialPort_LiDAR->BaudRate = 500000;
			serialPort_LiDAR->Open();
		}
		if (serialPort_LiDAR->IsOpen)
		{
			serialPort_LiDAR->Write(LMS_Angular_range_change_manage, 0, 11);
			_sleep(500);
			serialPort_LiDAR->Write(continuous_LMS_data_manage, 0, 8);
		}
		t1 = clock();
	}
	private: System::Void Btn_LiDAR_DisConnect_Click(System::Object^  sender, System::EventArgs^  e) {
		timer1->Stop();
		cli::array<System::Byte>^ LMS_Stope_manage = gcnew cli::array<System::Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x25, 0x38, 0x08};
		serialPort_LiDAR->Write(LMS_Stope_manage, 0, 8);
		serialPort_LiDAR->Close();
	}
	private: System::Void Btn_Refresh_Combox_Click(System::Object^  sender, System::EventArgs^  e) {
		ComPortRefresh();
	}
	private: System::Void Btn_RadarA_Connect_Click(System::Object^  sender, System::EventArgs^  e) {
		if (serialPort_Radar_Angle->IsOpen)serialPort_Radar_Angle->Close();
		targetDistant = Convert::ToDouble(txBox_targetDistant->Text) * 100;
		AlphaBias = Convert::ToDouble(txBox_AlphaBias->Text);
		serialPort_Radar_Angle->PortName = cBox_Radar_Angle->Text;
		serialPort_Radar_Angle->Encoding = System::Text::Encoding::GetEncoding(28591);
		serialPort_Radar_Angle->BaudRate = 19200;
		serialPort_Radar_Angle->DataBits = 8;
		serialPort_Radar_Angle->StopBits = StopBits::One;
		serialPort_Radar_Angle->Parity = Parity::None;
		serialPort_Radar_Angle->Open();
		_sleep(500);
		cli::array<byte>^ cmd = gcnew cli::array<System::Byte>{ 0x80, 0x64, 0xA1, 0x1F, 0x7C, 0x00, 0x00, 0xAB, 0xFD, 0x01, 0x00, 0xC9};
		serialPort_Radar_Angle->Write(cmd, 0, 12);
	}
	private: System::Void Btn_UpDateSetting_Click(System::Object^  sender, System::EventArgs^  e) {
		targetDistant = Convert::ToDouble(txBox_targetDistant->Text) * 100;
		PartitionValue = Convert::ToDouble(tBox_Partition->Text) * 100;
	}
	private: System::Void Btn_RadarAngle_DisConnect_Click(System::Object^  sender, System::EventArgs^  e) {
		chart1->Series["Series_Radar_Angle"]->Points->Clear();
		serialPort_Radar_Angle->Close();
	}
	private: System::Void Btn_LeftBias_Click(System::Object^  sender, System::EventArgs^  e) {
		Pt Rotation = CoordinateRotation(-35.0f, AngleRadar_Point);
		left_Radar_bias.x = LiDAR_tmpPt.x - Rotation.y;
		left_Radar_bias.y = LiDAR_tmpPt.y - Rotation.x;


		tx_LRadarBias_X->Text = Math::Round(left_Radar_bias.x, 2).ToString();
		tx_LRadarBias_Y->Text = Math::Round(left_Radar_bias.y, 2).ToString();
		f_getLRadarBias = true;
		std::fstream fp;
		fp.open("LBias.txt", std::ios::out);
		fp << left_Radar_bias.x << " " << left_Radar_bias.y;
		fp.close();
		ckBox_RadarR->Checked = false;
	}
	private: System::Void Btn_RightBias_Click(System::Object^  sender, System::EventArgs^  e) {

		ckBox_RadarR->Checked = true;
		Pt Rotation = CoordinateRotation(-125.0f, AngleRadar_Point);
		right_Radar_bias.x = LiDAR_tmpPt.x - Rotation.x;
		right_Radar_bias.y = LiDAR_tmpPt.y - Rotation.y;
		tx_RRadarBias_X->Text = Math::Round(right_Radar_bias.x, 2).ToString();
		tx_RRadarBias_Y->Text = Math::Round(right_Radar_bias.y, 2).ToString();
		f_getRRadarBias = true;
		std::fstream fp;
		fp.open("RBias.txt", std::ios::out);
		fp << right_Radar_bias.x << " " << right_Radar_bias.y;
		fp.close();
	}
#pragma endregion

#pragma region 聚類
	private:bool predicate(Pt P1, Pt P2)
	{
		double distant = Math::Sqrt(Math::Pow((P1.x - P2.x), 2) + Math::Pow((P1.y - P2.y), 2));
		return  distant <= PartitionValue;
	}
	private:int partition(cv::vector<Pt>& _vec, cv::vector<int>& labels)
	{
		int i, j, N = _vec.size();
		const Pt* vec = &_vec[0];

		const int PARENT = 0;
		const int RANK = 1;

		cv::vector<int> _nodes(N * 2);
		int(*nodes)[2] = (int(*)[2])&_nodes[0];

		for (i = 0; i < N; i++)
		{
			nodes[i][PARENT] = -1;
			nodes[i][RANK] = 0;
		}
		for (i = 0; i < N; i++)
		{
			int root = i;

			// find root
			while (nodes[root][PARENT] >= 0)
				root = nodes[root][PARENT];

			for (j = 0; j < N; j++)
			{
				if (i == j || !predicate(vec[i], vec[j]))
					continue;
				int root2 = j;

				while (nodes[root2][PARENT] >= 0)
					root2 = nodes[root2][PARENT];

				if (root2 != root)
				{
					// unite both trees
					int rank = nodes[root][RANK], rank2 = nodes[root2][RANK];
					if (rank > rank2)
						nodes[root2][PARENT] = root;
					else
					{
						nodes[root][PARENT] = root2;
						nodes[root2][RANK] += rank == rank2;
						root = root2;
					}
					//assert(nodes[root][PARENT] < 0);

					int k = j, parent;

					// compress the path from node2 to root
					while ((parent = nodes[k][PARENT]) >= 0)
					{
						nodes[k][PARENT] = root;
						k = parent;
					}

					// compress the path from node to root
					k = i;
					while ((parent = nodes[k][PARENT]) >= 0)
					{
						nodes[k][PARENT] = root;
						k = parent;
					}
				}
			}
		}
		for (unsigned int i = 0; i < N; i++)
			labels.push_back(0);
		int nclasses = 0;

		for (i = 0; i < N; i++)
		{
			int root = i;
			while (nodes[root][PARENT] >= 0)
				root = nodes[root][PARENT];
			if (nodes[root][RANK] >= 0)
				nodes[root][RANK] = ~nclasses++;
			labels[i] = ~nodes[root][RANK];
		}
		return nclasses;
	}
	private:vector<Pt> CaculateAveragePoint(vector<Pt>&XYcord, vector<int> &PointLab, int NoObj)
	{
		vector<Pt>averagePoint;
		for (uint i = 0; i < NoObj; i++)
		{
			int LabCounter = 0;
			Pt sum = Pt(0, 0);
			for (uint j = 0; j < XYcord.size(); j++)
			{
				if (PointLab[j] == i)
				{
					sum.x += XYcord[j].x;
					sum.y += XYcord[j].y;
					LIDAR_cooridate.erase(XYcord.begin() + j);
					PointLab.erase(PointLab.begin() + j);
					LabCounter++;
				}

			}
			sum.x /= LabCounter;
			sum.y /= LabCounter;
			averagePoint.push_back(sum);
		}
		return averagePoint;
	}
	private:void FindClosePoint(vector<Pt>&NewPoints, vector<Pt>&oldPoints, double timeInterval)
	{
		for (uint i = 0; i < NewPoints.size(); i++)
		{
			double minDistant = 8000000;
			double distant;
			for (uint j = 0; j < oldPoints.size(); j++)
			{
				distant = sqrt(pow(NewPoints[i].x-oldPoints[j].x, 2) + pow(NewPoints[i].y - oldPoints[j].y, 2));
				if (distant < minDistant)
				{
					minDistant = distant;
				}
			}
			NewPoints[i].velcity = distant / timeInterval;
		}
	}
#pragma endregion
	private:void LoadData()
	{
		std::fstream fp;
		fp.open("LBias.txt", std::ios::in);
		fp >> left_Radar_bias.x;
		fp >> left_Radar_bias.y;
		fp.close();
		fp.open("RBias.txt", std::ios::in);
		fp >> right_Radar_bias.x;
		fp >> right_Radar_bias.y;
		fp.close();
		tx_LRadarBias_X->Text = Math::Round(right_Radar_bias.x, 2).ToString();
		tx_LRadarBias_Y->Text = Math::Round(right_Radar_bias.y, 2).ToString();
		tx_RRadarBias_X->Text = Math::Round(left_Radar_bias.x, 2).ToString();
		tx_RRadarBias_Y->Text = Math::Round(left_Radar_bias.y, 2).ToString();
	}


	};
}
