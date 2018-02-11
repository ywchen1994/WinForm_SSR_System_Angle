#pragma once
#define _USE_MATH_DEFINES
#include <Windows.h>
#include"math.h"
#include<vector>
#include <time.h>
#include "Pt.h"
#include "CTBox.h"
#include <fstream>
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
	using namespace std;
	int LiDAR_Data[722];
	vector<Pt> LIDAR_cooridate;
	CTBox TBox;
	Pt LiDAR_tmpPt = Pt(0, 0);
	Pt right_Radar_bias;
	Pt left_Radar_bias;
	Pt AngleRadar_Point;
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
			ComPortRefresh();
			timer1->Interval = 100;
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
				delete[]LIDAR_X_cooridate;
				delete[]LIDAR_Y_cooridate;
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
		bool f_getRRadarBias = false;
		bool f_getLRadarBias = false;
		double *LIDAR_X_cooridate = new double[361];
		double *LIDAR_Y_cooridate = new double[361];
		double bsdAngle = 0;
		float AlphaBias;
		double targetDistant;
		uint format = 25;


#pragma region 視窗物件
	private: System::Windows::Forms::TabPage^  tabPage2;
	private: System::Windows::Forms::TabPage^  tabPage1;

	private: System::Windows::Forms::GroupBox^  groupBox2;
	private: System::Windows::Forms::TextBox^  txBox_AlphaBias;
	private: System::Windows::Forms::Label^  label1;
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
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::TextBox^  txBox_targetDistant;
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

	private: System::Windows::Forms::Label^  label6;
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
				 this->tabPage1 = (gcnew System::Windows::Forms::TabPage());
				 this->label7 = (gcnew System::Windows::Forms::Label());
				 this->chart1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
				 this->groupBox4 = (gcnew System::Windows::Forms::GroupBox());
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
				 this->label6 = (gcnew System::Windows::Forms::Label());
				 this->Btn_RadarAngle_DisConnect = (gcnew System::Windows::Forms::Button());
				 this->label3 = (gcnew System::Windows::Forms::Label());
				 this->txBox_targetDistant = (gcnew System::Windows::Forms::TextBox());
				 this->lbBsdAngleT = (gcnew System::Windows::Forms::Label());
				 this->label2 = (gcnew System::Windows::Forms::Label());
				 this->txBox_AlphaBias = (gcnew System::Windows::Forms::TextBox());
				 this->label1 = (gcnew System::Windows::Forms::Label());
				 this->Btn_RadarA_Connect = (gcnew System::Windows::Forms::Button());
				 this->cBox_Radar_Angle = (gcnew System::Windows::Forms::ComboBox());
				 this->Btn_Refresh_Combox = (gcnew System::Windows::Forms::Button());
				 this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
				 this->cBox_LiDAR = (gcnew System::Windows::Forms::ComboBox());
				 this->Btn_LiDAR_Connected = (gcnew System::Windows::Forms::Button());
				 this->Btn_LiDAR_DisConnect = (gcnew System::Windows::Forms::Button());
				 this->tabControl1 = (gcnew System::Windows::Forms::TabControl());
				 this->serialPort_Tbox = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->Btn_Tbox_Close = (gcnew System::Windows::Forms::Button());
				 this->tabPage1->SuspendLayout();
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
				 this->tabPage2->BackColor = System::Drawing::Color::DimGray;
				 this->tabPage2->Location = System::Drawing::Point(4, 25);
				 this->tabPage2->Margin = System::Windows::Forms::Padding(4);
				 this->tabPage2->Name = L"tabPage2";
				 this->tabPage2->Padding = System::Windows::Forms::Padding(4);
				 this->tabPage2->Size = System::Drawing::Size(1831, 780);
				 this->tabPage2->TabIndex = 1;
				 this->tabPage2->Text = L"圖";
				 // 
				 // tabPage1
				 // 
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
				 this->tabPage1->Size = System::Drawing::Size(1831, 780);
				 this->tabPage1->TabIndex = 0;
				 this->tabPage1->Text = L"設定";
				 this->tabPage1->UseVisualStyleBackColor = true;
				 // 
				 // label7
				 // 
				 this->label7->AutoSize = true;
				 this->label7->BackColor = System::Drawing::Color::White;
				 this->label7->Location = System::Drawing::Point(1636, 122);
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
				 this->chart1->Location = System::Drawing::Point(268, 4);
				 this->chart1->Margin = System::Windows::Forms::Padding(4);
				 this->chart1->Name = L"chart1";
				 series1->ChartArea = L"ChartArea1";
				 series1->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series1->Legend = L"Legend1";
				 series1->Name = L"Series_LiDAR";
				 series2->ChartArea = L"ChartArea1";
				 series2->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series2->Color = System::Drawing::Color::Yellow;
				 series2->Legend = L"Legend1";
				 series2->MarkerColor = System::Drawing::Color::Yellow;
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
				 this->chart1->Size = System::Drawing::Size(1600, 750);
				 this->chart1->TabIndex = 8;
				 this->chart1->Text = L"圖";
				 // 
				 // groupBox4
				 // 
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
				 this->groupBox2->Controls->Add(this->label6);
				 this->groupBox2->Controls->Add(this->Btn_RadarAngle_DisConnect);
				 this->groupBox2->Controls->Add(this->label3);
				 this->groupBox2->Controls->Add(this->txBox_targetDistant);
				 this->groupBox2->Controls->Add(this->lbBsdAngleT);
				 this->groupBox2->Controls->Add(this->label2);
				 this->groupBox2->Controls->Add(this->txBox_AlphaBias);
				 this->groupBox2->Controls->Add(this->label1);
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
				 // label6
				 // 
				 this->label6->AutoSize = true;
				 this->label6->Location = System::Drawing::Point(180, 176);
				 this->label6->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label6->Name = L"label6";
				 this->label6->Size = System::Drawing::Size(28, 15);
				 this->label6->TabIndex = 9;
				 this->label6->Text = L"(m)";
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
				 // label3
				 // 
				 this->label3->AutoSize = true;
				 this->label3->Location = System::Drawing::Point(11, 176);
				 this->label3->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label3->Name = L"label3";
				 this->label3->Size = System::Drawing::Size(87, 15);
				 this->label3->TabIndex = 7;
				 this->label3->Text = L"target Distant:";
				 // 
				 // txBox_targetDistant
				 // 
				 this->txBox_targetDistant->Location = System::Drawing::Point(111, 164);
				 this->txBox_targetDistant->Margin = System::Windows::Forms::Padding(4);
				 this->txBox_targetDistant->Name = L"txBox_targetDistant";
				 this->txBox_targetDistant->Size = System::Drawing::Size(60, 25);
				 this->txBox_targetDistant->TabIndex = 6;
				 this->txBox_targetDistant->Text = L"3.24";
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
				 // txBox_AlphaBias
				 // 
				 this->txBox_AlphaBias->Location = System::Drawing::Point(111, 114);
				 this->txBox_AlphaBias->Margin = System::Windows::Forms::Padding(4);
				 this->txBox_AlphaBias->Name = L"txBox_AlphaBias";
				 this->txBox_AlphaBias->ReadOnly = true;
				 this->txBox_AlphaBias->Size = System::Drawing::Size(60, 25);
				 this->txBox_AlphaBias->TabIndex = 3;
				 this->txBox_AlphaBias->Text = L"-2.84";
				 // 
				 // label1
				 // 
				 this->label1->AutoSize = true;
				 this->label1->Location = System::Drawing::Point(11, 126);
				 this->label1->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
				 this->label1->Name = L"label1";
				 this->label1->Size = System::Drawing::Size(73, 15);
				 this->label1->TabIndex = 2;
				 this->label1->Text = L"Alpha Bias:";
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
				 this->tabControl1->Location = System::Drawing::Point(31, 15);
				 this->tabControl1->Margin = System::Windows::Forms::Padding(4);
				 this->tabControl1->Name = L"tabControl1";
				 this->tabControl1->SelectedIndex = 0;
				 this->tabControl1->Size = System::Drawing::Size(1839, 809);
				 this->tabControl1->TabIndex = 0;
				 // 
				 // serialPort_Tbox
				 // 
				 this->serialPort_Tbox->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_Tbox_DataReceived);
				 // 
				 // Btn_Tbox_Close
				 // 
				 this->Btn_Tbox_Close->Location = System::Drawing::Point(136, 74);
				 this->Btn_Tbox_Close->Name = L"Btn_Tbox_Close";
				 this->Btn_Tbox_Close->Size = System::Drawing::Size(100, 29);
				 this->Btn_Tbox_Close->TabIndex = 12;
				 this->Btn_Tbox_Close->Text = L"關閉";
				 this->Btn_Tbox_Close->UseVisualStyleBackColor = true;
				 this->Btn_Tbox_Close->Click += gcnew System::EventHandler(this, &MyForm::Btn_Tbox_Close_Click);
				 // 
				 // MyForm
				 // 
				 this->AutoScaleDimensions = System::Drawing::SizeF(8, 15);
				 this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
				 this->AutoSizeMode = System::Windows::Forms::AutoSizeMode::GrowAndShrink;
				 this->ClientSize = System::Drawing::Size(1876, 839);
				 this->Controls->Add(this->tabControl1);
				 this->Margin = System::Windows::Forms::Padding(4);
				 this->Name = L"MyForm";
				 this->Text = L"MyForm";
				 this->tabPage1->ResumeLayout(false);
				 this->tabPage1->PerformLayout();
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
				TBox.L_RADAR_Speed = bTboxData[17] - 127;
				TBox.L_RADAR_Angle = bTboxData[18] - 127;
				TBox.L_RADAR_Point = Pt(TBox.L_RADAR_Range*Math::Cos(TBox.L_RADAR_Angle), TBox.L_RADAR_Range*Math::Sin(TBox.L_RADAR_Angle));

				TBox.R_RADAR_Mode = bTboxData[19];
				TBox.R_RADAR_ALert = bTboxData[20];
				TBox.R_RADAR_Range = bTboxData[21];
				TBox.R_RADAR_Speed = bTboxData[22] - 127;
				TBox.R_RADAR_Angle = bTboxData[23] - 127;
				TBox.R_RADAR_Point = Pt(TBox.R_RADAR_Range*Math::Cos(TBox.R_RADAR_Angle), TBox.R_RADAR_Range*Math::Sin(TBox.R_RADAR_Angle));
			}

		}
		if (serialPort_Tbox->BytesToRead >= format * 2)
		{
			serialPort_Tbox->DiscardInBuffer();

		}
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
	private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
		chart1->Series["Series_LiDAR"]->Points->Clear();
		chart1->Series["Series_LiDAR_CLOSE"]->Points->Clear();
		chart1->Series["Series_Radar_Angle"]->Points->Clear();
		chart1->Series["Series_TBox_RRadar"]->Points->Clear();
		chart1->Series["Series_TBox_LRadar"]->Points->Clear();
		lbBsdAngleT->Text = Math::Round(bsdAngle, 2).ToString();
		vector<Pt> Pt_average;
		if (f_getLiDARData)
		{
			LIDAR_cooridate.resize(361);
			for (uint i = 0; i < 361; i++)
			{
				LIDAR_cooridate[i] = Pt(LIDAR_X_cooridate[i], LIDAR_Y_cooridate[i]);
				chart1->Series["Series_LiDAR"]->Points->AddXY(LIDAR_X_cooridate[i], LIDAR_Y_cooridate[i]);
			}
			vector<int>Lab;
			int NoObj = partition(LIDAR_cooridate, Lab);

			Pt_average.resize(NoObj);
			for (uint i = 0; i < NoObj; i++)
			{
				int LabCounter = 0;
				Pt sum = Pt(0, 0);
				for (uint j = 0; j < LIDAR_cooridate.size(); j++)
				{
					if (Lab[j] == i)
					{
						sum.x += LIDAR_cooridate[j].x;
						sum.y += LIDAR_cooridate[j].y;
						LIDAR_cooridate.erase(LIDAR_cooridate.begin() + j);
						Lab.erase(Lab.begin() + j);
						LabCounter++;
					}

				}
				sum.x /= LabCounter;
				sum.y /= LabCounter;
				Pt_average[i] = sum;
			}
			Pt P;
			double minDistant = 100000;
			for (uint i = 0; i < Pt_average.size(); i++)
			{
				if (pow(Pt_average[i].x, 2) + pow(Pt_average[i].y, 2) < minDistant && pow(Pt_average[i].x, 2) + pow(Pt_average[i].y, 2) != 0)
				{
					minDistant = pow(Pt_average[i].x, 2) + pow(Pt_average[i].y, 2);
					P = Pt_average[i];
				}
			}
			time_t t2 = clock();
			float time = (float)(t2 - t1) / CLK_TCK;
			double speed = sqrt(pow(P.x - LiDAR_tmpPt.x, 2) + pow(P.y - LiDAR_tmpPt.y, 2)) / time;
			t1 = t2;
			LiDAR_tmpPt = P;

			chart1->Series["Series_LiDAR_CLOSE"]->Points->AddXY(P.x, P.y);
			chart1->Series["Series_LiDAR_CLOSE"]->Label = "(" + Math::Round(P.x, 2).ToString() + " , " + Math::Round(P.y, 2).ToString() + " , " + Math::Round(speed, 2).ToString() + ")";
		}
		Pt Radar_Angle_Point;
		if (ckBox_RadarR->Checked)
		{

			Pt Rotationtmp = CoordinateRotation(-35.0f, AngleRadar_Point);
			Radar_Angle_Point.x = Rotationtmp.y + right_Radar_bias.x;
			Radar_Angle_Point.y = Rotationtmp.x + right_Radar_bias.y;
		}
		else
		{
			Pt Rotation = CoordinateRotation(-125.0f, AngleRadar_Point);

			Radar_Angle_Point.x = Rotation.x + left_Radar_bias.x;
			Radar_Angle_Point.y = Rotation.y + left_Radar_bias.y;
		}
		Pt L_RadarPtAtLiDAR = L_Radar2LiDAR(Pt(100 * TBox.L_RADAR_Range*Math::Cos(TBox.L_RADAR_Angle), 100 * TBox.L_RADAR_Range*Math::Sin(TBox.L_RADAR_Angle)));
		Pt R_RadarPtAtLiDAR = R_Radar2LiDAR(Pt(100 * TBox.R_RADAR_Range*Math::Cos(TBox.R_RADAR_Angle), 100 * TBox.R_RADAR_Range*Math::Sin(TBox.R_RADAR_Angle)));
		Tx_CarSpeed->Text = TBox.currentSpeed.ToString();
		Tx_Radar_Mode->Text = "L:" + getRadarMode(TBox.L_RADAR_Mode) + "  R:" + getRadarMode(TBox.R_RADAR_Mode);

		chart1->Series["Series_TBox_RRadar"]->Points->AddXY(R_RadarPtAtLiDAR.x, R_RadarPtAtLiDAR.y);
		chart1->Series["Series_TBox_LRadar"]->Points->AddXY(L_RadarPtAtLiDAR.x, L_RadarPtAtLiDAR.y);
		chart1->Series["Series_Radar_Angle"]->Points->AddXY(Radar_Angle_Point.x, Radar_Angle_Point.y);
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
	private:void ComPortRefresh(void)
	{
		cBox_LiDAR->Items->Clear();
		cBox_Radar_Angle->Items->Clear();
		cBox_TBox->Items->Clear();
		cli::array<System::String^>^ Port = SerialPort::GetPortNames();
		cBox_LiDAR->Items->AddRange(Port);
		cBox_Radar_Angle->Items->AddRange(Port);
		cBox_TBox->Items->AddRange(Port);
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
		cBox_LiDAR->Items->Clear();
		cli::array<System::String^>^ Port = SerialPort::GetPortNames();
		cBox_LiDAR->Items->AddRange(Port);
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
	private: System::Void Btn_RadarAngle_DisConnect_Click(System::Object^  sender, System::EventArgs^  e) {
		serialPort_Radar_Angle->Close();
	}

	private: System::Void Btn_LeftBias_Click(System::Object^  sender, System::EventArgs^  e) {
		Pt Rotation = CoordinateRotation(-125.0f, AngleRadar_Point);
		left_Radar_bias.x = LiDAR_tmpPt.x - Rotation.x;
		left_Radar_bias.y = LiDAR_tmpPt.y - Rotation.y;
		tx_LRadarBias_X->Text = Math::Round(left_Radar_bias.x, 2).ToString();
		tx_LRadarBias_Y->Text = Math::Round(left_Radar_bias.y, 2).ToString();
		f_getLRadarBias = true;
		fstream fp;
		fp.open("LBias.txt", ios::out);
		fp << left_Radar_bias.x << " " << left_Radar_bias.y;
		fp.close();
	}
	private: System::Void Btn_RightBias_Click(System::Object^  sender, System::EventArgs^  e) {
		Pt Rotation = CoordinateRotation(-35.0f, AngleRadar_Point);
		right_Radar_bias.x = LiDAR_tmpPt.x - Rotation.y;
		right_Radar_bias.y = LiDAR_tmpPt.y - Rotation.x;


		tx_RRadarBias_X->Text = Math::Round(right_Radar_bias.x, 2).ToString();
		tx_RRadarBias_Y->Text = Math::Round(right_Radar_bias.y, 2).ToString();
		f_getRRadarBias = true;
		fstream fp;
		fp.open("RBias.txt", ios::out);
		fp << right_Radar_bias.x << " " << right_Radar_bias.y;
		fp.close();
		ckBox_RadarR->Checked = true;
	}
#pragma endregion
	private:Pt L_Radar2LiDAR(Pt P)
	{
		Pt Ans;
		Pt Rotation = CoordinateRotation(-125, P);
		Ans.x = Rotation.x + left_Radar_bias.x;
		Ans.y = Rotation.y + left_Radar_bias.y;
		return Ans;
	}
	private:Pt R_Radar2LiDAR(Pt P)
	{
		Pt Ans;
		Pt Rotationtmp = CoordinateRotation(-35.0f, P);
		Ans.x = Rotationtmp.y + right_Radar_bias.x;
		Ans.y = Rotationtmp.x + right_Radar_bias.y;
		return Ans;
	}
#pragma region 聚類
	private:bool predicate(Pt P1, Pt P2)
	{
		return ((P1.x - P2.x)*(P1.x - P2.x) + (P1.y - P2.y)*(P1.y - P2.y)) <= 100;
	}
	private:int partition(vector<Pt>& _vec, vector<int>& labels)
	{
		int i, j, N = _vec.size();
		const Pt* vec = &_vec[0];

		const int PARENT = 0;
		const int RANK = 1;

		vector<int> _nodes(N * 2);
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
#pragma endregion


	private: System::Void Btn_Tbox_Close_Click(System::Object^  sender, System::EventArgs^  e) {
		serialPort_Tbox->Close();
	}
	private:void LoadData()
	{
		fstream fp;
		fp.open("RBias.txt",ios::in);
		fp >> right_Radar_bias.x;
		fp >> right_Radar_bias.y;
		fp.close();
		fp.open("LBias.txt", ios::in);
		fp >>left_Radar_bias.x;
		fp >> left_Radar_bias.y;
		fp.close();
	}
};
}
