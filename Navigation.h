#pragma once

#include <vector>
#include <unordered_map>

class CNode;

class CNodeData
{
public:

	std::vector<CNode*> m_vChilds;
	CNode* m_pParent = nullptr;
	float m_fF = 0.f, m_fG = 0.f, m_fH = 0.f;

	CNodeData()
	{
		m_vChilds.clear();
	}
	~CNodeData() {}

};

class CNode
{
public:

	CNodeData* m_pData = nullptr;
	float m_fX = 0.f, m_fY = 0.f, m_fZ = 0.f;
	int m_iChilds = 0;
	int m_iID = -1;

	CNode() {}
	CNode(int iID, float fX, float fY, float fZ)
	{
		m_pData = new CNodeData;
		m_iID = iID;
		m_fX = fX;
		m_fY = fY;
		m_fZ = fZ;
	}

	void SetData(CNode* pParent, float fG, float fH);
};

class CNavFile
{
private:


public:

	std::unordered_map<int, std::vector<int>> m_mapChilds;
	std::vector<CNode*> m_vNodes;
	char m_szName[64];
	FILE* m_pFile = nullptr;

	CNavFile() {}
	~CNavFile() {}

	bool Create(FILE** pFile, const char* szFileName);
	bool Open(FILE** pFile, const char* szFileName);
	bool Save();
	bool Close();

	void CreateNode(float fX, float fY, float fZ);
	void AttachNodes(CNode* pChild, CNode* pParent);
	void GenerateMap(std::vector<CNode*>& vNodes);

};

class AStar
{
private:


public:

	AStar() {}
	~AStar() {}

	CNode* m_pStart = nullptr;
	CNode* m_pEnd = nullptr;
	std::vector<CNode*> m_vOpenSet;
	std::vector<CNode*> m_vClosedSet;
	std::vector<CNode*> m_vPath;
	std::vector<CNode*> m_vNodes;
	bool m_bFound = false;
	float m_fLastDistance = 0.f;

	enum eHeuristicMethod
	{
		EUCLIDEAN,
		MANHATTAN
	};

	CNavFile* LoadNavigationMap(const char* szFileName);

	void BackTrackPath(CNode* pCurrentNode);

	CNode* GetNearestNodeFromPosition(const glm::vec3& vPosition);

	float HeuristicCost(const CNode* p1, const CNode* p2, const eHeuristicMethod& eMethod = EUCLIDEAN) const;
	float GetDistance(const CNode* pNode1, const CNode* pNode2) const;

	bool CalculatePath(CNode* pStart = nullptr, CNode* pEnd = nullptr);

};

class CNavigationMapping
{
public:

	CNavigationMapping();
	~CNavigationMapping() {}

	CNavFile* CreateNavigationFile(const char* szFileName);
	CNavFile* OpenNavigationFile(const char* szFileName);


};


extern std::unique_ptr<AStar> g_AStar;
extern std::unique_ptr<CNavigationMapping> g_Navigation;
