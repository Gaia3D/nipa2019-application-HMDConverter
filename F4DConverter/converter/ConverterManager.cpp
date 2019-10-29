// ./converter/ConverterManager.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "ConverterManager.h"

#include <direct.h>
#include <io.h>
#include <sys/stat.h>
#include <algorithm>

#include "proj_api.h"

#ifdef POINTCLOUDFORMAT
#include "cpl_serv.h"
#endif

#include "../argumentDefinition.h"
#include "./reader/ReaderFactory.h"
#include "./process/ConversionProcessor.h"
#include "./process/SceneControlVariables.h"
#include "./writer/F4DWriter.h"
#include "./util/utility.h"
#include "./util/json/json.h"

#include "LogWriter.h"


// CConverterManager

CConverterManager CConverterManager::m_ConverterManager;

CConverterManager::CConverterManager()
{
	processor = NULL;

	bCreateIndices =  bConversion = false;

	bOcclusionCulling = false;

	unitScaleFactor = 1.0;

	skinLevel = 3;

	bYAxisUp = false;

	bAlignPostionToCenter = false;

	meshType = 0;

	bUseReferenceLonLat = false;

	bUseEpsg = false;

	offsetX = offsetY = offsetZ = 0.0;
}

CConverterManager::~CConverterManager()
{
	if(processor != NULL)
		delete processor;
}


// CConverterManager 멤버 함수
#ifdef POINTCLOUDFORMAT
#ifdef _DEBUG
#pragma comment(lib, "../external/geotiff/lib/geotiff_d.lib")
#else
#pragma comment(lib, "../external/geotiff/lib/geotiff.lib")
#endif
std::string csvFullPath;
static const char* CSVFileFullPathOverride(const char* baseFile)
{
	static char szPath[1024];

	sprintf(szPath, "%s/%s", csvFullPath.c_str(), baseFile);

	return szPath;
}
#endif

bool CConverterManager::initialize(std::map<std::string, std::string>& arguments)
{
	std::string programPath = arguments[ProgramPath];
	std::string programFolder;

	size_t lastSlashPos = programPath.rfind('/');
	programFolder = programPath.substr(0, lastSlashPos + 1);
	std::string projLibPath = programFolder + std::string("proj");
	const char* epsgPath = projLibPath.c_str();
	pj_set_searchpath(1, &epsgPath);

	if (!setProcessConfiguration(arguments))
	{
		pj_set_searchpath(0, NULL);
		return false;
	}

#ifdef POINTCLOUDFORMAT
	csvFullPath = programFolder + std::string("csv");

	SetCSVFilenameHook(CSVFileFullPathOverride);
#endif

	if(processor == NULL)
		processor = new ConversionProcessor();

	return processor->initialize();
}

void CConverterManager::uninitialize()
{
	pj_set_searchpath(0, NULL);

	if (processor != NULL)
		processor->uninitialize();
}

bool CConverterManager::processDataFolder()
{
	std::string inputFolder = inputFolderPath;
	std::string outputFolder = outputFolderPath;
	// test if output folder exist
	bool outputFolderExist = false;
	if (_access(outputFolder.c_str(), 0) == 0)
	{
		struct stat status;
		stat(outputFolder.c_str(), &status);
		if (status.st_mode & S_IFDIR)
			outputFolderExist = true;
	}

	if (!outputFolderExist)
	{
		LogWriter::getLogWriter()->addContents(std::string(ERROR_FLAG), false);
		LogWriter::getLogWriter()->addContents(std::string(NO_DATA_OR_INVALID_PATH), false);
		LogWriter::getLogWriter()->addContents(outputFolder, true);
		return false;
	}

	bool inputFolderExist = false;
	if (_access(inputFolder.c_str(), 0) == 0)
	{
		struct stat status;
		stat(inputFolder.c_str(), &status);
		if (status.st_mode & S_IFDIR)
			inputFolderExist = true;
	}

	if (!inputFolderExist)
	{
		LogWriter::getLogWriter()->addContents(std::string(ERROR_FLAG), false);
		LogWriter::getLogWriter()->addContents(std::string(NO_DATA_OR_INVALID_PATH), false);
		LogWriter::getLogWriter()->addContents(inputFolder, true);
		return false;
	}

	std::map<std::string, std::string> targetFiles;
	collectTargetFiles(inputFolder, targetFiles);
	if (targetFiles.empty())
	{
		LogWriter::getLogWriter()->addContents(std::string(ERROR_FLAG), false);
		LogWriter::getLogWriter()->addContents(std::string(NO_DATA_OR_INVALID_PATH), false);
		LogWriter::getLogWriter()->addContents(inputFolder, true);
		return false;
	}

	if (!projectName.empty())
	{
		outputFolder = outputFolder + std::string("/") + projectName;
		bool bProjectFolderExist = false;
		if (_access(outputFolder.c_str(), 0) == 0)
		{
			struct stat status;
			stat(outputFolder.c_str(), &status);
			if (status.st_mode & S_IFDIR)
				bProjectFolderExist = true;
		}

		if (!bProjectFolderExist)
		{
			if (_mkdir(outputFolder.c_str()) != 0)
			{
				LogWriter::getLogWriter()->addContents(std::string(ERROR_FLAG), false);
				LogWriter::getLogWriter()->addContents(std::string(CANNOT_CREATE_DIRECTORY), false);
				LogWriter::getLogWriter()->addContents(outputFolder, true);
				return false;
			}
		}

		outputFolderPath = outputFolder;
	}

	processDataFiles(targetFiles);

	return true;
}

void CConverterManager::processDataFiles(std::map<std::string, std::string>& targetFiles)
{

	// TODO(khj 20180417) : NYI setup conversion configuration here
	// now, only set wheter do occlusion culling or not
	processor->setVisibilityIndexing(bOcclusionCulling);
	processor->setSkinLevel(skinLevel);
	//processor->setAlignPostionToCenter(bAlignPostionToCenter);
	processor->setMeshType(meshType);
	switch (meshType)
	{
	case 1: // single random-shaped 3d mesh
		processor->setSkinLevel(50);
		processor->setLeafSpatialOctreeSize(40.0f);
		break;
	case 2: // splitted random-shaped 3d mesh
		processor->setSkinLevel(51);
		processor->setLeafSpatialOctreeSize(40.0f);
		break;
	case 3: // point cloud
		processor->setLeafSpatialOctreeSize(40.0f);
		break;
	}
	// TODO(khj 20180417) end

	//// hard-cord for japan(AIST) realistic mesh and romania data
	//processor->setVisibilityIndexing(false);
	/*processor->setYAxisUp(false);
	processor->setAlignPostionToCenter(bAlignPostionToCenter);
	processor->setMeshType(meshType);
	switch(meshType)
	{
	case 1:
		processor->setSkinLevel(50);
		break;
	case 2:
		processor->setSkinLevel(51);
		break;
	}
	processor->setLeafSpatialOctreeSize(40.0f);*/

	//// hard-cord for new york citygml
	//processor->setVisibilityIndexing(false);
	//processor->setUseNsm(false);
	//processor->setYAxisUp(true);
	//processor->setAlignPostionToCenter(true);
	//processor->setLeafSpatialOctreeSize(422.0f);

	std::map<std::string, double> centerXs, centerYs;
	processSingleLoop(targetFiles, centerXs, centerYs, 0);

	// save representative lon / lat of F4D if a reference file exists
	if (!centerXs.empty() && !centerYs.empty())
	{
		writeRepresentativeLonLatOfEachData(centerXs, centerYs);
	}
}

bool CConverterManager::writeIndexFile()
{
	F4DWriter writer(NULL);
	writer.setWriteFolder(outputFolderPath);
	writer.writeIndexFile();

	return true;
}

void CConverterManager::processSingleLoop(std::map<std::string, std::string>& targetFiles,
										std::map<std::string, double>& centerXs,
										std::map<std::string, double>& centerYs,
										unsigned char depth)
{
	std::string outputFolder = outputFolderPath;

	std::string fullId;
	std::map<std::string, std::string>::iterator iter = targetFiles.begin();
	for (; iter != targetFiles.end(); iter++)
	{
		std::string dataFile = iter->first;
		std::string dataFileFullPath = iter->second;

		aReader* reader = ReaderFactory::makeReader(dataFileFullPath);
		if (reader == NULL)
			continue;

		printf("\n===== Start processing this file : %s\n", dataFile.c_str());

		if(depth == 0)
			LogWriter::getLogWriter()->numberOfFilesToBeConverted += 1;

		reader->setUnitScaleFactor(unitScaleFactor);
		reader->setOffset(offsetX, offsetY, offsetZ);
		reader->setYAxisUp(bYAxisUp);

		// 0. inject coordinate information into reader before reading
		if (bUseEpsg)
			reader->injectSrsInfo(epsgCode);

		if (bUseReferenceLonLat)
			reader->injectOringinInfo(referenceLon, referenceLat);

		// 1. read the original file and build data structure
		if (!reader->readRawDataFile(dataFileFullPath))
		{
			LogWriter::getLogWriter()->addContents(std::string(ERROR_FLAG), false);
			LogWriter::getLogWriter()->addContents(std::string(CANNOT_LOAD_FILE), false);
			LogWriter::getLogWriter()->addContents(dataFileFullPath, true);
			printf("[ERROR]%s\n", std::string(CANNOT_LOAD_FILE).c_str());
			delete reader;
			continue;
		}

		// 1-1. in cases when original files are converted into multiple temporary files.
		if (!reader->getTemporaryFiles().empty())
		{
			printf("===== %zd temporary files are made. Proceeding conversion of temporary files\n", reader->getTemporaryFiles().size());

			// run recursively
			processSingleLoop(reader->getTemporaryFiles(), centerXs, centerYs, depth + 1);

			// delete temporary files
			std::map<std::string, std::string>::iterator tmpFileIter = reader->getTemporaryFiles().begin();
			for (; tmpFileIter != reader->getTemporaryFiles().end(); tmpFileIter++)
			{
				if (remove(tmpFileIter->second.c_str()) != 0)
				{
					LogWriter::getLogWriter()->addContents(std::string(CANNOT_DELETE_FILE), false);
					LogWriter::getLogWriter()->addContents(tmpFileIter->second, true);
				}
			}

			delete reader;

			continue;
		}

		// 1-2. basic data reading validation
		bool bGeometryExists = true;
		if (reader->getDataContainer().empty())
		{
			if (reader->getMultipleDataContainers().empty())
				bGeometryExists = false;
			else
			{
				std::map<std::string, std::vector<gaia3d::TrianglePolyhedron*>>::iterator subItemIter = reader->getMultipleDataContainers().begin();
				bGeometryExists = false;
				for (; subItemIter != reader->getMultipleDataContainers().end(); subItemIter++)
				{
					if (!iter->second.empty())
					{
						bGeometryExists = true;
						break;
					}
				}
			}
		}

		if (!bGeometryExists)
		{
			LogWriter::getLogWriter()->addContents(std::string(ERROR_FLAG), false);
			LogWriter::getLogWriter()->addContents(std::string(NO_DATA_IN_RAW_DATA), false);
			LogWriter::getLogWriter()->addContents(dataFileFullPath, true);
			printf("[ERROR]%s\n", std::string(NO_DATA_IN_RAW_DATA).c_str());
			delete reader;
			continue;
		}

		// 2. convert to F4D
		std::map<std::string, std::vector<gaia3d::TrianglePolyhedron*>> targetOriginalGeometries;
		if (reader->shouldRawDataBeConvertedToMuitiFiles())
			targetOriginalGeometries = reader->getMultipleDataContainers();
		else
		{
			std::string::size_type dotPosition = dataFile.rfind(".");
			std::string id = dataFile.substr(0, dotPosition);

			targetOriginalGeometries[id] = reader->getDataContainer();
		}

		std::map<std::string, std::vector<std::string>>& ancestorsOfEachItem = reader->getAncestorsOfEachSubGroup();

		std::map<std::string, std::vector<gaia3d::TrianglePolyhedron*>>::iterator subItemIter = targetOriginalGeometries.begin();
		for (; subItemIter != targetOriginalGeometries.end(); subItemIter++)
		{
			if (subItemIter->second.empty())
				continue;

			if (reader->shouldGeometryBeDesroyedOutside())
				processor->setResponsibilityForDisposing(false);

			fullId = subItemIter->first;
			if (!idPrefix.empty())
				fullId = idPrefix + fullId;
			if (!idSuffix.empty())
				fullId = fullId + idSuffix;

			if (reader->shouldRawDataBeConvertedToMuitiFiles())
				printf("\n===== Start processing sub-group : %s\n", fullId.c_str());

			// 2.1 create directories suiting for hiararchy
			std::string finalOutputFolder = outputFolder;
			if (ancestorsOfEachItem.find(subItemIter->first) != ancestorsOfEachItem.end())
			{
				bool bCanMakeSubDirectory = true;
				for (size_t i = ancestorsOfEachItem[subItemIter->first].size(); i > 0; i--)
				{
					finalOutputFolder = finalOutputFolder + std::string("/F4D_") + idPrefix + ancestorsOfEachItem[subItemIter->first][i-1] + idSuffix;

					bool bFinalOutputFolder = false;
					if (_access(finalOutputFolder.c_str(), 0) == 0)
					{
						struct stat status;
						stat(finalOutputFolder.c_str(), &status);
						if (status.st_mode & S_IFDIR)
							bFinalOutputFolder = true;
					}

					if (!bFinalOutputFolder)
					{
						if (_mkdir(finalOutputFolder.c_str()) != 0)
						{
							LogWriter::getLogWriter()->addContents(std::string(ERROR_FLAG), false);
							LogWriter::getLogWriter()->addContents(std::string(CANNOT_CREATE_DIRECTORY), false);
							LogWriter::getLogWriter()->addContents(finalOutputFolder, true);
							bCanMakeSubDirectory = false;
							break;
						}
					}
				}

				if (!bCanMakeSubDirectory)
				{
					if (reader->shouldRawDataBeConvertedToMuitiFiles())
						printf("\n===== End processing sub-group : %s\n", fullId.c_str());

					continue;
				}
			}

			// 2.2 conversion
			if (!processor->proceedConversion(subItemIter->second, reader->getTextureInfoContainer()))
			{
				LogWriter::getLogWriter()->addContents(std::string(ERROR_FLAG), false);
				LogWriter::getLogWriter()->addContents(std::string(CONVERSION_FAILURE), false);
				printf("[ERROR]%s\n", std::string(CONVERSION_FAILURE).c_str());
				if (reader->shouldRawDataBeConvertedToMuitiFiles())
					LogWriter::getLogWriter()->addContents(dataFileFullPath + std::string("--") + fullId, true);
				else
					LogWriter::getLogWriter()->addContents(dataFileFullPath, true);

				processor->clear();

				if (reader->shouldRawDataBeConvertedToMuitiFiles())
					printf("\n===== End processing sub-group : %s\n", fullId.c_str());

				continue;
			}

			// 2.3 get representative lon/lat of original dataset
			if (reader->doesHasGeoReferencingInfo())
			{
				double lon, lat;
				reader->getGeoReferencingInfo(lon, lat);
				centerXs[fullId] = lon;
				centerYs[fullId] = lat;
			}

			processor->addAttribute(std::string(F4DID), fullId);

			// 2.4 write
			F4DWriter writer(processor);
			writer.setWriteFolder(finalOutputFolder);
			if (!writer.write())
			{
				if (reader->shouldRawDataBeConvertedToMuitiFiles())
					LogWriter::getLogWriter()->addContents(dataFileFullPath + std::string("--") + fullId, true);
				else
					LogWriter::getLogWriter()->addContents(dataFileFullPath, true);
			}

			processor->clear();

			if (reader->shouldRawDataBeConvertedToMuitiFiles())
				printf("===== End processing sub-group : %s\n", fullId.c_str());
		}

		delete reader;

		if (depth == 0)
			LogWriter::getLogWriter()->numberOfFilesConverted += 1;

		printf("===== End processing this file : %s\n", dataFile.c_str());
	}
}

bool CConverterManager::setProcessConfiguration(std::map<std::string, std::string>& arguments)
{
	if (arguments.find(InputFolder) != arguments.end())
	{
		bConversion = true;
		inputFolderPath = arguments[InputFolder];

		if (arguments.find(MeshType) != arguments.end())
		{
			meshType = std::stoi(arguments[MeshType]);
		}

		if (arguments.find(PerformOC) != arguments.end())
		{
			if (arguments[PerformOC] == std::string("Y") || arguments[PerformOC] == std::string("y"))
				bOcclusionCulling = true;
			else
				bOcclusionCulling = false;
		}
		else
			bOcclusionCulling = false;

		if (arguments.find(UnitScaleFactor) != arguments.end())
			unitScaleFactor = std::stod(arguments[UnitScaleFactor]);

		if (arguments.find(SkinLevelNsm) != arguments.end())
			skinLevel = (unsigned char)(unsigned int)std::stoi(arguments[SkinLevelNsm]);

		if (arguments.find(IdPrefix) != arguments.end())
			idPrefix = arguments[IdPrefix];

		if (arguments.find(IdSuffix) != arguments.end())
			idSuffix = arguments[IdSuffix];

		if (arguments.find(IsYAxisUp) != arguments.end())
		{
			if (arguments[IsYAxisUp] == std::string("Y") ||
				arguments[IsYAxisUp] == std::string("y"))
				bYAxisUp = true;
			else
				bYAxisUp = false;
		}

		if (arguments.find(ReferenceLonLat) != arguments.end())
		{
			size_t lonLatLength = arguments[ReferenceLonLat].length();
			char* original = new char[lonLatLength + 1];
			memset(original, 0x00, sizeof(char)*(lonLatLength + 1));
			memcpy(original, arguments[ReferenceLonLat].c_str(), lonLatLength);
			char* lon = std::strtok(original, ",");
			char* lat = std::strtok(NULL, ",");
			referenceLon = std::stod(lon);
			referenceLat = std::stod(lat);
			delete[] original;
			bUseReferenceLonLat = true;
		}

		if (arguments.find(AlignToCenter) != arguments.end())
		{
			if (arguments[AlignToCenter] == std::string("Y") ||
				arguments[AlignToCenter] == std::string("y"))
				bAlignPostionToCenter = true;
			else
				bAlignPostionToCenter = false;
		}

		if (arguments.find(Epsg) != arguments.end())
		{
			epsgCode = arguments[Epsg];

			std::string proj4String = std::string("+init=epsg:") + epsgCode;

			projPJ pjEpsg;
			pjEpsg = pj_init_plus(proj4String.c_str());
			if (pjEpsg == NULL)
			{
				char* errorMsg = pj_strerrno(pj_errno);
				LogWriter::getLogWriter()->addContents(std::string(UNSUPPERTED_EPSG_CODE), false);
				LogWriter::getLogWriter()->addContents(epsgCode, true);

				return false;
			}

			bUseEpsg = true;
		}

		if (arguments.find(OffsetX) != arguments.end())
			offsetX = std::stod(arguments[OffsetX]);

		if (arguments.find(OffsetY) != arguments.end())
			offsetY = std::stod(arguments[OffsetY]);

		if (arguments.find(OffsetZ) != arguments.end())
			offsetY = std::stod(arguments[OffsetZ]);

		if (arguments.find(ProjectName) != arguments.end())
			projectName = arguments[ProjectName];
	}
	else
		bConversion = false;

	if (arguments.find(OutputFolder) != arguments.end())
	{
		outputFolderPath = arguments[OutputFolder];
	}

	if (arguments.find(CreateIndex) != arguments.end())
	{
		if (arguments[CreateIndex] == std::string("Y") ||
			arguments[CreateIndex] == std::string("y"))
			bCreateIndices = true;
		else
			bCreateIndices = false;
	}
	else
		bCreateIndices = false;

	return true;
}

void CConverterManager::process()
{
	if (bConversion)
	{
		if (processor->getSceneControlVariables()->m_width == 0 || processor->getSceneControlVariables()->m_height == 0 ||
			processor->getSceneControlVariables()->m_myhDC == 0 || processor->getSceneControlVariables()->m_hRC == 0)
			return;

		processDataFolder();
	}

	if (bCreateIndices)
		writeIndexFile();
}

void CConverterManager::collectTargetFiles(std::string& inputFolder, std::map<std::string, std::string>& targetFiles)
{
	_finddata_t fd;
	long long handle;
	int result = 1;
	std::string fileFilter = inputFolder + std::string("/*.*");
	handle = _findfirst(fileFilter.c_str(), &fd);

	if (handle == -1)
		return;

	std::vector<std::string> subFolders;
	while (result != -1)
	{
		if ((fd.attrib & _A_SUBDIR) == _A_SUBDIR)
		{
			if (std::string(fd.name) != std::string(".") && std::string(fd.name) != std::string(".."))
			{
				std::string subFolder = std::string(fd.name);
#ifdef _WIN32
				subFolder = gaia3d::StringUtility::convertMultibyteToUtf8(subFolder);
#endif
				std::string subFolderFullPath = inputFolder + "/" + subFolder;
				subFolders.push_back(subFolderFullPath);
			}
		}
		else
		{
			std::string dataFile = std::string(fd.name);
#ifdef _WIN32
			dataFile = gaia3d::StringUtility::convertMultibyteToUtf8(dataFile);
#endif

			// WARNING!!! this part is customized for HMD
			std::string::size_type dotPosition = dataFile.rfind(".");
			if (dotPosition == std::string::npos)
			{
				result = _findnext(handle, &fd);
				continue;
			}

			std::string::size_type fileExtLength = dataFile.length() - dotPosition - 1;
			std::string fileExt = dataFile.substr(dotPosition + 1, fileExtLength);
			std::transform(fileExt.begin(), fileExt.end(), fileExt.begin(), towlower);

			if (fileExt.compare(std::string("rev")) != 0)
			{
				result = _findnext(handle, &fd);
				continue;
			}

			std::string tempDataFile = dataFile;
			std::transform(tempDataFile.begin(), tempDataFile.end(), tempDataFile.begin(), towlower);
			if (tempDataFile.find("_hull.rev") != std::string::npos)
			{
				result = _findnext(handle, &fd);
				continue;
			}
			// WARNING end

			std::string dataFileFullPath = inputFolder + std::string("/") + dataFile;
			targetFiles[dataFile] = dataFileFullPath;
		}

		result = _findnext(handle, &fd);
	}

	_findclose(handle);

	size_t subFolderCount = subFolders.size();
	for (size_t i = 0; i < subFolderCount; i++)
	{
		collectTargetFiles(subFolders[i], targetFiles);
	}
}

void CConverterManager::writeRepresentativeLonLatOfEachData(std::map<std::string, double>& posXs, std::map<std::string, double>& posYs)
{
	Json::Value arrayNode(Json::arrayValue);

	std::map<std::string, double>::iterator iter = posXs.begin();
	for (; iter != posXs.end(); iter++)
	{
		Json::Value f4d(Json::objectValue);

		// data_key
		std::string dataKey = iter->first;
		f4d["data_key"] = dataKey;

		// longitude and latitude
		f4d["longitude"] = iter->second;
		f4d["latitude"] = posYs[iter->first];

		arrayNode.append(f4d);
	}

	Json::StyledWriter writer;
	std::string documentContent = writer.write(arrayNode);
	std::string lonLatFileFullPath = outputFolderPath + std::string("/lonsLats.json");
	FILE* file = NULL;
	file = fopen(lonLatFileFullPath.c_str(), "wt");
	fprintf(file, "%s", documentContent.c_str());
	fclose(file);
}