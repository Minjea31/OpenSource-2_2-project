// RosBridge_MFCDlg.h: 헤더 파일
//

#pragma once
#include "afxsock.h" // 소켓 라이브러리 포함
#include <opencv2/opencv.hpp>  // OpenCV 라이브러리
#include <vector>

#include <iostream>

#pragma comment(lib, "ws2_32.lib")

// 이미지 수신 소켓 클래스 (JPEG 데이터 처리)
class CMyImageReceiver : public CAsyncSocket {
public:
	CMyImageReceiver(CWnd* pParent = nullptr) : m_pParent(pParent) {}

	virtual void OnReceive(int nErrorCode);

	void SetParent(CWnd* pParent) { m_pParent = pParent; } // 부모 설정 함수 추가

private:
	CWnd* m_pParent;
};


// --------------------------------------
// 서버 역할 클래스
// --------------------------------------
class Server {
public:
	Server(CWnd* pParent = nullptr);
	~Server();
	void InitServer(int port);
	void AcceptAndReceive();
	void CloseServer();

private:
	CWnd* m_pParent;
	SOCKET m_serverSocket;
	SOCKET m_clientSocket;
	sockaddr_in m_serverAddr;
	sockaddr_in m_clientAddr;
	int m_clientAddrLen;
};





// CRosBridgeMFCDlg 대화 상자
class CRosBridgeMFCDlg : public CDialogEx
{
	// 생성입니다.
public:
	CRosBridgeMFCDlg(CWnd* pParent = nullptr);	// 표준 생성자입니다.

	CAsyncSocket m_socket; // 클라이언트 소켓
	CMyImageReceiver m_imageReceiver; // 이미지 수신 소켓 추가


	bool m_bcurruse;        // 버튼 클릭 여부 확인
	COLORREF m_circleColor = RGB(0, 255, 0); // 초기 색상: 초록색


	cv::Mat m_receivedImage; // 수신된 이미지를 저장할 버퍼
	bool m_isImageUpdated;   // 이미지 업데이트 상태 플래그

	CBitmap m_bitmap;      // 비트맵 객체
	BITMAP  m_bitmapInfo;  // 비트맵 정보




	// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ROSBRIDGE_MFC_DIALOG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.


	// 구현입니다.
protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnClickedSendButton1();
	afx_msg void OnDestroy();
	afx_msg void OnClickedSendButon2();

	void DrawCircle(CDC* pDC, CRect rect, COLORREF color); // 원 그리기 함수
	void UpdateImage(const cv::Mat& image); // 이미지를 업데이트하는 함수
};