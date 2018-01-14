#include <thread>
#include <memory.h>
#include <algorithm>
#include <vector>
#include <unordered_map>

#include "../Console/Console.h"
#include "../glm.h"
#include "../InternalGlobalVars/IGVars.h"
#include "Navigation.h"

std::unique_ptr<AStar> g_AStar;
std::unique_ptr<CNavigationMapping> g_Navigation;

/*
FREE MEMORY NOT DONE
*/

void CNode::SetData(CNode* pParent, float fG, float fH)
{
	m_pData->m_pParent = pParent;
	m_pData->m_fG = fG;
	m_pData->m_fH = fH;
	m_pData->m_fF = fG + fH;
}

CNavFile* AStar::LoadNavigationMap(const char* szFileName)
{
	CNavFile* pNavFile = g_Navigation->OpenNavigationFile(szFileName);

	m_vOpenSet.clear();
	m_vClosedSet.clear();
	m_vPath.clear();
	m_vNodes.clear();

	pNavFile->GenerateMap(m_vNodes);

	m_bFound = false;

	return pNavFile;
}

float AStar::HeuristicCost(const CNode* p1, const CNode* p2, const eHeuristicMethod& eMethod) const
{
	return eMethod == MANHATTAN ? abs(p1->m_fX - p2->m_fX) + abs(p1->m_fY - p2->m_fY) + abs(p1->m_fZ - p2->m_fZ) :
		sqrtf((p2->m_fX - p1->m_fX) *
		(p2->m_fX - p1->m_fX) +
			(p2->m_fY - p1->m_fY) *
			(p2->m_fY - p1->m_fY) +
			(p2->m_fZ - p1->m_fZ) *
			(p2->m_fZ - p1->m_fZ));
}

float AStar::GetDistance(const CNode* pNode1, const CNode* pNode2) const
{
	return sqrtf((pNode2->m_fX - pNode1->m_fX) *
		(pNode2->m_fX - pNode1->m_fX) +
		(pNode2->m_fY - pNode1->m_fY) *
		(pNode2->m_fY - pNode1->m_fY) +
		(pNode2->m_fZ - pNode1->m_fZ) *
		(pNode2->m_fZ - pNode1->m_fZ));
}

void AStar::BackTrackPath(CNode* pCurrentNode)
{
	CNode* pTemp = pCurrentNode;
	m_vPath.push_back(pTemp);

	m_fLastDistance = 0.f;

	while (pTemp->m_pData->m_pParent)
	{
		m_fLastDistance += pTemp->m_pData->m_pParent->m_pData->m_fF;
		m_vPath.push_back(pTemp->m_pData->m_pParent);
		pTemp = pTemp->m_pData->m_pParent;
	}
}

CNode* AStar::GetNearestNodeFromPosition(const glm::vec3& vPosition)
{
	CNode* pFoundNode = nullptr;
	float fDist = FLT_MAX, fActualDist = FLT_MAX;

	std::for_each(m_vNodes.begin(), m_vNodes.end(), [&](CNode* pNode)
	{
		fActualDist = glm::distance(vPosition, glm::vec3(pNode->m_fX, pNode->m_fY, pNode->m_fZ));
		if (fActualDist < fDist)
		{
			fDist = fActualDist;
			pFoundNode = pNode;
		}
	});

	return pFoundNode;
}

bool AStar::CalculatePath(CNode* pStart, CNode* pEnd)
{
	if (!pStart || !pEnd)
	{
		m_pStart = m_vNodes[0];
		m_pEnd = m_vNodes[m_vNodes.size() - 1];
	}
	else
	{
		m_pStart = pEnd;
		m_pEnd = pStart;
	}

	m_vOpenSet.clear();
	m_vClosedSet.clear();
	m_vPath.clear();

	m_vOpenSet.push_back(m_pStart);

	while (m_vOpenSet.size() > 0)
	{
		CNode* pCurrent = *m_vOpenSet.begin();

		std::for_each(m_vOpenSet.begin(), m_vOpenSet.end(), [&](CNode* pNode)
		{
			if (pNode->m_pData->m_fF < pCurrent->m_pData->m_fF)
				pCurrent = pNode;
		});

		if (pCurrent->m_iID == m_pEnd->m_iID)
		{
			BackTrackPath(pCurrent);
			return (m_bFound = true);
		}

		m_vOpenSet.erase(std::find(m_vOpenSet.begin(), m_vOpenSet.end(), pCurrent));
		m_vClosedSet.push_back(pCurrent);

		for (int i = 0; i < pCurrent->m_pData->m_vChilds.size(); ++i)
		{
			CNode* pChild = pCurrent->m_pData->m_vChilds[i];

			if (std::find(m_vClosedSet.begin(), m_vClosedSet.end(), pChild) != m_vClosedSet.end())
				continue;

			const float fTempG = pCurrent->m_pData->m_fG + GetDistance(pCurrent, pChild);

			if (std::find(m_vOpenSet.begin(), m_vOpenSet.end(), pChild) == m_vOpenSet.end())
				m_vOpenSet.push_back(pChild);
			else if (fTempG >= pChild->m_pData->m_fG)
				continue;

			pChild->SetData(pCurrent, fTempG, HeuristicCost(pChild, m_pEnd));
		}
	}

	return (m_bFound = false);
}

void CNavFile::GenerateMap(std::vector<CNode*>& vNodes)
{
	std::for_each(m_vNodes.begin(), m_vNodes.end(), [&](CNode* pNode)
	{
		CNode* pNewNode = new CNode(pNode->m_iID, pNode->m_fX, pNode->m_fY, pNode->m_fZ);
		vNodes.push_back(pNewNode);

		std::for_each(m_mapChilds[pNode->m_iID].begin(), m_mapChilds[pNode->m_iID].end(), [&](const int& pChildID)
		{
			pNewNode->m_pData->m_vChilds.push_back(m_vNodes[pChildID]);
		});
	});

	m_vNodes.clear();
}

bool CNavFile::Create(FILE** pFile, const char* szFileName)
{
	fopen_s(pFile, ("Data//paths//" + std::string(szFileName) + ".nav").c_str(), "wb");

	m_pFile = pFile ? *pFile : nullptr;

	strcpy_s(m_szName, szFileName);

	return pFile ? true : false;
}

bool CNavFile::Open(FILE** pFile, const char* szFileName)
{
	if (m_pFile)
		fclose(m_pFile);

	m_vNodes.clear();
	m_mapChilds.clear();
	m_pFile = nullptr;

	fopen_s(pFile, ("Data//paths//" + std::string(szFileName) + ".nav").c_str(), "rb");

	m_pFile = pFile ? *pFile : nullptr;

	strcpy_s(m_szName, szFileName);

	if (m_pFile)
	{
		int uiNodes = 0;
		fread(&uiNodes, sizeof(int), 1, m_pFile);
		for (int i = 0; i < uiNodes; ++i)
		{
			CNode* pNode = new CNode;
			fread(pNode, sizeof(CNode), 1, m_pFile);
			if (!pNode)
				continue;

			m_vNodes.push_back(pNode);

			if (pNode->m_iChilds > 0)
			{
				for (int j = 0; j < pNode->m_iChilds; ++j)
				{
					int pChildID = -1;
					fread(&pChildID, sizeof(int), 1, m_pFile);

					if (pChildID <= -1)
						continue;

					m_mapChilds[pNode->m_iID].push_back(pChildID);
				}
			}
		}

		std::for_each(m_vNodes.begin(), m_vNodes.end(), [&](CNode* pNode)
		{
			std::for_each(m_mapChilds[pNode->m_iID].begin(), m_mapChilds[pNode->m_iID].end(), [&](const int& pChild)
			{
				if (!pNode->m_pData)
					pNode->m_pData = new CNodeData;

				pNode->m_pData->m_vChilds.push_back(m_vNodes[pChild]);
			});
		});
		return true;
	}
	return false;
}

bool CNavFile::Save()
{
	fclose(m_pFile);
	fopen_s(&m_pFile, ("Data//paths//" + std::string(m_szName) + ".nav").c_str(), "wb");

	if (m_pFile)
	{
		const int uiNodes = m_vNodes.size();
		fwrite(&uiNodes, sizeof(int), 1, m_pFile);
		std::for_each(m_vNodes.begin(), m_vNodes.end(), [&](CNode* pNode)
		{
			pNode->m_pData = nullptr;
			fwrite(pNode, sizeof(CNode), 1, m_pFile);
			std::for_each(m_mapChilds[pNode->m_iID].begin(), m_mapChilds[pNode->m_iID].end(), [&](int pChildID)
			{
				fwrite(&pChildID, sizeof(int), 1, m_pFile);
			});
		});
		return true;
	}
	return false;
}

bool CNavFile::Close()
{
	if (m_pFile)
		fclose(m_pFile);
	return true;
}

void CNavFile::CreateNode(float fX, float fY, float fZ)
{
	CNode* pTempNode = new CNode(m_vNodes.size(), fX, fY, fZ);
	m_vNodes.push_back(pTempNode);
}

void CNavFile::AttachNodes(CNode* pChild, CNode* pParent)
{
	m_mapChilds[pParent->m_iID].push_back(pChild->m_iID);
	m_mapChilds[pChild->m_iID].push_back(pParent->m_iID);
	++pParent->m_iChilds;
	++pChild->m_iChilds;
}

CNavigationMapping::CNavigationMapping()
{
	g_AStar = std::make_unique<AStar>();
}

CNavFile* CNavigationMapping::CreateNavigationFile(const char* szFileName)
{
	CNavFile* pFileData = new CNavFile;
	FILE* pFile = nullptr;

	if (!pFileData->Create(&pFile, szFileName))
	{
		g_Console->Print(DColor::Red, __FILE__, __FUNCTION__, "Could not create new navigation file '%s'", szFileName);
		return nullptr;
	}
	return pFileData;
}

CNavFile* CNavigationMapping::OpenNavigationFile(const char* szFileName)
{
	CNavFile* pFileData = new CNavFile;
	FILE* pFile = nullptr;

	if (!pFileData->Open(&pFile, szFileName))
	{
		g_Console->Print(DColor::Red, __FILE__, __FUNCTION__, "Could not open new navigation file '%s'", szFileName);
		return nullptr;
	}
	return pFileData;
}