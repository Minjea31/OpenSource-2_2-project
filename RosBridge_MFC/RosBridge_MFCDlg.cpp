// RosBridge_MFCDlg.cpp: 구현 파일
//

#include "pch.h"
#include "framework.h"
#include "RosBridge_MFCDlg.h"
#include "RosBridge_MFC.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define ROS_SERVER_IP _T("192.168.0.193")
#define ROS_SERVER_PORT 65432


// --------------------------------------
// CAboutDlg 대화 상자 (정보 대화 상자)
// --------------------------------------
class CAboutDlg : public CDialogEx {
public:
	CAboutDlg();

	// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX) {}

void CAboutDlg::DoDataExchange(CDataExchange* pDX) {
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CRosBridgeMFCDlg 대화 상자



// 생성자
CRosBridgeMFCDlg::CRosBridgeMFCDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_ROSBRIDGE_MFC_DIALOG, pParent),
	m_bcurruse(false), m_circleColor(RGB(0, 255, 0)) {

	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

// 데이터 교환
void CRosBridgeMFCDlg::DoDataExchange(CDataExchange* pDX) {
	CDialogEx::DoDataExchange(pDX);
}

// --------------------------------------
// CRosBridgeMFCDlg 대화 상자 구현
// --------------------------------------
BEGIN_MESSAGE_MAP(CRosBridgeMFCDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_SEND_BUTTON1, &CRosBridgeMFCDlg::OnClickedSendButton1)
	ON_BN_CLICKED(IDC_SEND_BUTON2, &CRosBridgeMFCDlg::OnClickedSendButon2)
	ON_WM_DESTROY()
END_MESSAGE_MAP()


// CRosBridgeMFCDlg 메시지 처리기

BOOL CRosBridgeMFCDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.

	if (!AfxSocketInit()) {
		AfxMessageBox(_T("소켓 라이브러리 초기화 실패"));
		return FALSE;
	}

	m_imageReceiver.SetParent(this);

	AfxMessageBox(_T("서버와 클라이언트 초기화 중..."));

	// 비트맵 로드
	if (m_bitmap.LoadBitmap(IDB_BITMAP1)) {
		m_bitmap.GetBitmap(&m_bitmapInfo);
	}
	else {
		AfxMessageBox(_T("비트맵 로드 실패!"));
	}

	return TRUE;  // TRUE를 반환하면 포커스를 컨트롤에 설정하지 않음
}

void CRosBridgeMFCDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 애플리케이션의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

// 페인트 이벤트
void CRosBridgeMFCDlg::OnPaint() {
	if (IsIconic()) {
		CPaintDC dc(this);
		dc.DrawIcon(GetSystemMetrics(SM_CXICON) / 2, GetSystemMetrics(SM_CYICON) / 2, m_hIcon);
	}
	else {
		CPaintDC dc(this);

		if (!m_receivedImage.empty()) {
			cv::Mat rgbImage;
			cv::cvtColor(m_receivedImage, rgbImage, cv::COLOR_BGR2RGB);
			BITMAPINFO bitmapInfo = {};
			bitmapInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
			bitmapInfo.bmiHeader.biWidth = rgbImage.cols;
			bitmapInfo.bmiHeader.biHeight = -rgbImage.rows; // 상하 반전
			bitmapInfo.bmiHeader.biPlanes = 1;
			bitmapInfo.bmiHeader.biBitCount = 24;
			bitmapInfo.bmiHeader.biCompression = BI_RGB;

			dc.SetStretchBltMode(HALFTONE);
			StretchDIBits(
				dc.GetSafeHdc(),
				0, 0, rgbImage.cols, rgbImage.rows,
				0, 0, rgbImage.cols, rgbImage.rows,
				rgbImage.data, &bitmapInfo, DIB_RGB_COLORS, SRCCOPY
			);
		}

		// 비트맵 출력
		if (m_bitmap.m_hObject) {
			CDC memDC;
			memDC.CreateCompatibleDC(&dc);
			CBitmap* pOldBitmap = memDC.SelectObject(&m_bitmap);
			dc.BitBlt(60, 150, m_bitmapInfo.bmWidth, m_bitmapInfo.bmHeight, &memDC, 0, 0, SRCCOPY);
			memDC.SelectObject(pOldBitmap);
		}

		// 원 그리기
		CRect circleRect(120, 680, 160, 720);
		DrawCircle(&dc, circleRect, m_circleColor);

		CDialogEx::OnPaint();
	}
}


// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CRosBridgeMFCDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



// **클라이언트 메시지 전송 버튼 1**
void CRosBridgeMFCDlg::OnClickedSendButton1() {
	if (m_bcurruse) {
		AfxMessageBox(_T("이미 동작 중입니다."));
		return;
	}

	// 서버 연결 확인
	if (!m_socket.Create()) {
		AfxMessageBox(_T("소켓 생성 실패"));
		return;
	}

	if (!m_socket.Connect(ROS_SERVER_IP, ROS_SERVER_PORT)) {
		AfxMessageBox(_T("서버에 연결할 수 없습니다. 버튼을 다시 확인해 주세요."));
		m_socket.Close();
		return;
	}

	// 메시지 전송
	CString message = _T("1");
	if (m_socket.Send(message, message.GetLength() * sizeof(TCHAR)) == SOCKET_ERROR) {
		AfxMessageBox(_T("메시지 전송 실패"));
		m_socket.Close();
		return;
	}

	AfxMessageBox(_T("메시지 1 전송 완료"));
	m_socket.Close();

	m_circleColor = RGB(255, 0, 0);
	Invalidate();
	m_bcurruse = true;
}

// **클라이언트 메시지 전송 버튼 2**
void CRosBridgeMFCDlg::OnClickedSendButon2() {
	if (m_bcurruse) {
		AfxMessageBox(_T("이미 동작 중입니다."));
		return;
	}

	// 서버 연결 확인
	if (!m_socket.Create()) {
		AfxMessageBox(_T("소켓 생성 실패"));
		return;
	}

	if (!m_socket.Connect(ROS_SERVER_IP, ROS_SERVER_PORT)) {
		AfxMessageBox(_T("서버에 연결할 수 없습니다. 버튼을 다시 확인해 주세요."));
		m_socket.Close();
		return;
	}

	// 메시지 전송
	CString message = _T("2");
	if (m_socket.Send(message, message.GetLength() * sizeof(TCHAR)) == SOCKET_ERROR) {
		AfxMessageBox(_T("메시지 전송 실패"));
		m_socket.Close();
		return;
	}

	AfxMessageBox(_T("메시지 2 전송 완료"));
	m_socket.Close();

	m_circleColor = RGB(255, 0, 0);
	Invalidate();
	m_bcurruse = true;
}


// **종료 이벤트**
void CRosBridgeMFCDlg::OnDestroy() {
	CDialogEx::OnDestroy();
	m_socket.Close();
	AfxMessageBox(_T("서버 종료 완료."));
}



void CMyImageReceiver::OnReceive(int nErrorCode) {
	if (nErrorCode != 0) return;

	char buffer[65536] = { 0 };
	int bytesReceived = Receive(buffer, sizeof(buffer));
	if (bytesReceived > 0) {
		std::vector<uchar> jpegData(buffer, buffer + bytesReceived);
		int32_t receivedValue = *reinterpret_cast<int32_t*>(jpegData.data());

		CRosBridgeMFCDlg* pDlg = (CRosBridgeMFCDlg*)m_pParent;
		if (receivedValue == 1) {
			pDlg->m_circleColor = RGB(0, 255, 0);
			pDlg->m_bcurruse = false;
			pDlg->Invalidate();
		}

		cv::Mat image = cv::imdecode(jpegData, cv::IMREAD_COLOR);
		if (!image.empty()) {
			pDlg->UpdateImage(image);
			AfxMessageBox(_T("Image received and updated."));
		}
	}
}




// 원 그리기 함수
void CRosBridgeMFCDlg::DrawCircle(CDC* pDC, CRect rect, COLORREF color) {
	CPen pen(PS_SOLID, 2, RGB(0, 0, 0));
	CBrush brush(color);
	CPen* pOldPen = pDC->SelectObject(&pen);
	CBrush* pOldBrush = pDC->SelectObject(&brush);
	pDC->Ellipse(rect);
	pDC->SelectObject(pOldPen);
	pDC->SelectObject(pOldBrush);
}

// 이미지 업데이트 함수
void CRosBridgeMFCDlg::UpdateImage(const cv::Mat& image) {
	m_receivedImage = image.clone();
	Invalidate();
}

void Server::InitServer(int port) {
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		AfxMessageBox(_T("WSAStartup failed"));
		return;
	}

	m_serverSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (m_serverSocket == INVALID_SOCKET) {
		AfxMessageBox(_T("Socket creation failed"));
		WSACleanup();
		return;
	}

	m_serverAddr.sin_family = AF_INET;
	m_serverAddr.sin_addr.s_addr = INADDR_ANY;
	m_serverAddr.sin_port = htons(port);

	if (bind(m_serverSocket, (sockaddr*)&m_serverAddr, sizeof(m_serverAddr)) == SOCKET_ERROR) {
		AfxMessageBox(_T("Bind failed"));
		CloseServer();
		return;
	}

	if (listen(m_serverSocket, 3) == SOCKET_ERROR) {
		AfxMessageBox(_T("Listen failed"));
		CloseServer();
		return;
	}

	AfxMessageBox(_T("Server initialized and listening for connections."));
}

void Server::AcceptAndReceive() {
	while (true) {
		m_clientAddrLen = sizeof(m_clientAddr);
		m_clientSocket = accept(m_serverSocket, (sockaddr*)&m_clientAddr, &m_clientAddrLen);

		if (m_clientSocket == INVALID_SOCKET) {
			AfxMessageBox(_T("Accept failed."));
			break;
		}
		AfxMessageBox(_T("Client connected."));

		while (true) {
			int image_size = 0;
			if (recv(m_clientSocket, (char*)&image_size, sizeof(image_size), 0) <= 0) {
				AfxMessageBox(_T("Client disconnected."));
				break;
			}

			image_size = ntohl(image_size);
			std::vector<uchar> jpegData(image_size);
			int total_received = 0;

			while (total_received < image_size) {
				int bytes_received = recv(m_clientSocket, (char*)jpegData.data() + total_received, image_size - total_received, 0);
				if (bytes_received <= 0) break;
				total_received += bytes_received;
			}

			if (total_received != image_size) {
				AfxMessageBox(_T("Incomplete data received."));
				continue;
			}

			// 이미지 디코딩 및 업데이트
			cv::Mat image = cv::imdecode(jpegData, cv::IMREAD_COLOR);
			if (!image.empty() && m_pParent) {
				((CRosBridgeMFCDlg*)m_pParent)->UpdateImage(image);
				AfxMessageBox(_T("Image successfully received and updated."));
			}
			else {
				AfxMessageBox(_T("Failed to decode image."));
			}
		}
		closesocket(m_clientSocket);
		m_clientSocket = INVALID_SOCKET;
	}
}

void Server::CloseServer() {
	if (m_serverSocket != INVALID_SOCKET) {
		closesocket(m_serverSocket);
		WSACleanup();
		m_serverSocket = INVALID_SOCKET;
	}
}