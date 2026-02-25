#include "RVLPlatform.h"
#include <string.h>
#include "SceneSegFile.hpp"

namespace SceneSegFile
{
	//Parameter set dictionary for a list of paramater sets 
	std::map<int, std::string> FeatureGroupsDictionary::dictionary = {
		{ FeatureGroupsList::AdjacencyFeatureGroup, "AdjacencyFeatureGroup" }
	};

	//Parameter set dictionary for a list of paramater sets (Used only for names during SceneSegFile creation)
	std::map<int, std::string> FeatureSetsDictionary::dictionary = {
		{ FeatureSetsList::AdjacencyNode, "AdjacencyNode" }
	};

	//Parameter set dictionary for general parameters (Used only for names during SceneSegFile creation)
	std::map<int, std::string> FeaturesDictionary::dictionary = {
		{ FeaturesList::Centroid, "Centroid" },
		{ FeaturesList::NormalAngleDifference, "NormalAngleDifference" },
		{ FeaturesList::CupysFeature, "CupysFeature" },
		{ FeaturesList::GTObjectID, "GTObjectID" },
		{ FeaturesList::SameGTObject, "SameGTObject" },
		{ FeaturesList::PixelAffiliation, "PixelAffiliation" },
		{ FeaturesList::GTObjHistogram, "GTObjHistogram" },
		{ FeaturesList::CommonBoundaryLength, "CommonBoundaryLength" },
		{ FeaturesList::Vertices3D, "Vertices3D" },
		{ FeaturesList::FalseSegmentationCost, "FalseSegmentationCost" }
	};

	SceneSegFile* GenerateTestSceneSegFile()
	{
		//SceneSegFile object
		SceneSegFile* ssf = new SceneSegFile("Scene000");
		//1. element
		std::shared_ptr<SegFileElement> element = std::make_shared<SegFileElement>(0, "Element1");
		ssf->AddElement(element);
		//Adding stuff to element
		//Features
		element->features.AddFeature(FeaturesList::Centroid, FeaturesDictionary::dictionary.at(FeaturesList::Centroid), "float");
		float centroid[3] = { 1.1, 4.4, 5.5 };
		element->features.CopyFeatureData<float>(FeaturesList::Centroid, centroid, 3);
		//
		element->features.AddFeature(FeaturesList::CupysFeature, FeaturesDictionary::dictionary.at(FeaturesList::CupysFeature), "int");
		int cupy[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
		element->features.CopyFeatureData<int>(FeaturesList::CupysFeature, cupy, 10);
		//Feature sets
		element->AddFeatureSet(FeatureSetsList::AdjacencyNode, FeatureSetsDictionary::dictionary.at(FeatureSetsList::AdjacencyNode));
		std::shared_ptr<FeatureSet> featureset = element->featureSets.at(FeatureSetsList::AdjacencyNode);
		//feature set's features
		featureset->AddFeature(FeaturesList::Centroid, FeaturesDictionary::dictionary.at(FeaturesList::Centroid), "float");
		featureset->CopyFeatureData<float>(FeaturesList::Centroid, centroid, 3);
		//
		featureset->AddFeature(FeaturesList::CupysFeature, FeaturesDictionary::dictionary.at(FeaturesList::CupysFeature), "int");
		featureset->CopyFeatureData<int>(FeaturesList::CupysFeature, cupy, 10);
		//Feature groups
		element->AddFeatureGroup(FeatureGroupsList::AdjacencyFeatureGroup, FeatureGroupsDictionary::dictionary.at(FeatureGroupsList::AdjacencyFeatureGroup));
		std::shared_ptr<FeatureGroup> featuregroup = element->featureGroups.at(FeatureGroupsList::AdjacencyFeatureGroup);
		//feature group's feature set
		featuregroup->AddFeatureSet(FeatureSetsList::AdjacencyNode, FeatureSetsDictionary::dictionary.at(FeatureSetsList::AdjacencyNode));
		std::shared_ptr<FeatureSet> featuresetG = featuregroup->featureSets.at(FeatureSetsList::AdjacencyNode);
		//features
		featuresetG->AddFeature(FeaturesList::Centroid, FeaturesDictionary::dictionary.at(FeaturesList::Centroid), "float");
		featuresetG->CopyFeatureData<float>(FeaturesList::Centroid, centroid, 3);
		//
		featuresetG->AddFeature(FeaturesList::CupysFeature, FeaturesDictionary::dictionary.at(FeaturesList::CupysFeature), "int");
		featuresetG->CopyFeatureData<int>(FeaturesList::CupysFeature, cupy, 10);

		//2. element
		std::shared_ptr<SegFileElement> element2 = std::make_shared<SegFileElement>(1, "Element2");
		ssf->AddElement(element2);
		//Adding stuff to element
		//Features
		element2->features.AddFeature(FeaturesList::Centroid, FeaturesDictionary::dictionary.at(FeaturesList::Centroid), "float");
		element2->features.CopyFeatureData<float>(FeaturesList::Centroid, centroid, 3);
		//
		element2->features.AddFeature(FeaturesList::CupysFeature, FeaturesDictionary::dictionary.at(FeaturesList::CupysFeature), "int");
		element2->features.CopyFeatureData<int>(FeaturesList::CupysFeature, cupy, 10);
		//Feature sets
		element2->AddFeatureSet(FeatureSetsList::AdjacencyNode, FeatureSetsDictionary::dictionary.at(FeatureSetsList::AdjacencyNode));
		std::shared_ptr<FeatureSet> featureset2 = element2->featureSets.at(FeatureSetsList::AdjacencyNode);
		//feature set's features
		featureset2->AddFeature(FeaturesList::Centroid, FeaturesDictionary::dictionary.at(FeaturesList::Centroid), "float");
		featureset2->CopyFeatureData<float>(FeaturesList::Centroid, centroid, 3);
		//
		featureset2->AddFeature(FeaturesList::CupysFeature, FeaturesDictionary::dictionary.at(FeaturesList::CupysFeature), "int");
		featureset2->CopyFeatureData<int>(FeaturesList::CupysFeature, cupy, 10);
		//Feature groups
		element2->AddFeatureGroup(FeatureGroupsList::AdjacencyFeatureGroup, FeatureGroupsDictionary::dictionary.at(FeatureGroupsList::AdjacencyFeatureGroup));
		std::shared_ptr<FeatureGroup> featuregroup2 = element2->featureGroups.at(FeatureGroupsList::AdjacencyFeatureGroup);
		//feature group's feature set
		featuregroup2->AddFeatureSet(FeatureSetsList::AdjacencyNode, FeatureSetsDictionary::dictionary.at(FeatureSetsList::AdjacencyNode));
		std::shared_ptr<FeatureSet> featuresetG2 = featuregroup2->featureSets.at(FeatureSetsList::AdjacencyNode);
		//feature
		featuresetG2->AddFeature(FeaturesList::Centroid, FeaturesDictionary::dictionary.at(FeaturesList::Centroid), "float");
		featuresetG2->CopyFeatureData<float>(FeaturesList::Centroid, centroid, 3);
		//
		featuresetG2->AddFeature(FeaturesList::CupysFeature, FeaturesDictionary::dictionary.at(FeaturesList::CupysFeature), "int");
		featuresetG2->CopyFeatureData<int>(FeaturesList::CupysFeature, cupy, 10);
		//
		featuresetG2->AddFeature(FeaturesList::NormalAngleDifference, FeaturesDictionary::dictionary.at(FeaturesList::NormalAngleDifference), "string");
		std::string txt = "lalalala";
		featuresetG2->SetFeatureData<std::string>(FeaturesList::NormalAngleDifference, &txt);

		return ssf;
	}
}