#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <memory>

//RapidXML
#ifdef RVLLINUX
#include "rapidxml_ext.h"
#else
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#endif

namespace SceneSegFile
{
	//Base class for feature types
	class FeatureType
	{
	public:
		std::string featureName;
		std::string featureType;
		int size;
		bool idx;	//(if the id is index or dict id)
		FeatureType(std::string name, std::string t) : featureName(name), featureType(t) { size = 0; idx = false; }
		~FeatureType(){}
		virtual std::string GetDataAsString() = 0;// { return ""; }
		virtual void SetData(void*, int) = 0;
		virtual void CopyData(void*, int) = 0;
		virtual void* GetDataPtr() = 0;
	};

	//Feature type "int"
	class FeatureTypeInt : public FeatureType
	{
	public:
		int *data;
		FeatureTypeInt(std::string name, std::string t) : FeatureType(name, t) { this->data = NULL; }
		~FeatureTypeInt()
		{
			if (this->data)		//If exists, purge
				delete[] this->data;
		}
		std::string GetDataAsString()
		{
			std::stringstream ss(std::stringstream::in | std::stringstream::out);
			//Iterating through parameters
			for (int i = 0; i < this->size; i++)
				ss << this->data[i] << " ";

			std::string tempStr = ss.str();

			if (this->size > 0)
				//removig last 'space'
				tempStr.erase(tempStr.end() - 1);	//Check?

			return tempStr;
		}
		void SetData(void* data, int size)
		{
			this->data = (int*)data;
			this->size = size;
		}
		void CopyData(void* data, int size)
		{
			if (this->data)		//If exists, purge
				delete[] this->data;
			this->data = new int[size];	//Generate new data buffer and copy
			memcpy(this->data, data, size * sizeof(int));
			this->size = size;
		}
		void* GetDataPtr()
		{
			return this->data;
		}
		FeatureTypeInt& operator = (const FeatureTypeInt& other)
		{
			if (this != &other) //selfcheck
			{
				this->featureName = other.featureName;
				this->featureType = other.featureType;
				if (this->data)		//If exists, purge
					delete[] this->data;
				this->data = new int[other.size];	//Generate new data buffer and copy
				memcpy(this->data, other.data, other.size * sizeof(int));
				this->size = other.size;
				this->idx = other.idx;
			}
			return *this;
		}
	};

	//Feature type "float"
	class FeatureTypeFloat : public FeatureType
	{
	public:
		float *data;
		FeatureTypeFloat(std::string name, std::string t) : FeatureType(name, t) { this->data = NULL; }
		~FeatureTypeFloat() 
		{ 
			if (this->data)		//If exists, purge
				delete[] this->data;
		}
		std::string GetDataAsString()
		{
			std::stringstream ss(std::stringstream::in | std::stringstream::out);
			//Iterating through parameters
			for (int i = 0; i < this->size; i++)
				ss << this->data[i] << " ";
			//removig last 'space'
			std::string tempStr = ss.str();
			tempStr.erase(tempStr.end() - 1);	//Check?
			return tempStr;
		}
		void SetData(void* data, int size)
		{
			this->data = (float*)data;
			this->size = size;
		}
		void CopyData(void* data, int size)
		{
			if (this->data)		//If exists, purge
				delete[] this->data;
			this->data = new float[size];	//Generate new data buffer and copy
			memcpy(this->data, data, size * sizeof(float));
			this->size = size;
		}
		void* GetDataPtr()
		{
			return this->data;
		}
		FeatureTypeFloat& operator = (const FeatureTypeFloat& other)
		{
			if (this != &other) //selfcheck
			{
				this->featureName = other.featureName;
				this->featureType = other.featureType;
				if (this->data)		//If exists, purge
					delete[] this->data;
				this->data = new float[other.size];	//Generate new data buffer and copy
				memcpy(this->data, other.data, other.size * sizeof(float));
				this->size = other.size;
				this->idx = other.idx;
			}
			return *this;
		}
	};

	//Feature type "double"
	class FeatureTypeDouble : public FeatureType
	{
	public:
		double *data;
		FeatureTypeDouble(std::string name, std::string t) : FeatureType(name, t) { this->data = NULL; }
		~FeatureTypeDouble()
		{
			if (this->data)		//If exists, purge
				delete[] this->data;
		}
		std::string GetDataAsString()
		{
			std::stringstream ss(std::stringstream::in | std::stringstream::out);
			//Iterating through parameters
			for (int i = 0; i < this->size; i++)
				ss << this->data[i] << " ";
			//removig last 'space'
			std::string tempStr = ss.str();
			tempStr.erase(tempStr.end() - 1);	//Check?
			return tempStr;
		}
		void SetData(void* data, int size)
		{
			this->data = (double*)data;
			this->size = size;
		}
		void CopyData(void* data, int size)
		{
			if (this->data)		//If exists, purge
				delete[] this->data;
			this->data = new double[size];	//Generate new data buffer and copy
			memcpy(this->data, data, size * sizeof(double));
			this->size = size;
		}
		void* GetDataPtr()
		{
			return this->data;
		}
		FeatureTypeDouble& operator = (const FeatureTypeDouble& other)
		{
			if (this != &other) //selfcheck
			{
				this->featureName = other.featureName;
				this->featureType = other.featureType;
				if (this->data)		//If exists, purge
					delete[] this->data;
				this->data = new double[other.size];	//Generate new data buffer and copy
				memcpy(this->data, other.data, other.size * sizeof(double));
				this->size = other.size;
				this->idx = other.idx;
			}
			return *this;
		}
	};

	//Feature type "bool"
	class FeatureTypeBool : public FeatureType
	{
	public:
		bool data;
		FeatureTypeBool(std::string name, std::string t) : FeatureType(name, t) {}
		std::string GetDataAsString()
		{
			std::stringstream ss(std::stringstream::in | std::stringstream::out);
			ss << this->data;
			return ss.str();
		}
		void SetData(void* data, int size = 1)
		{
			this->data = *(bool*)data;
			this->size = 1;
		}
		void CopyData(void* data, int size)
		{
			this->data = *(bool*)data;
			this->size = 1;
		}
		void* GetDataPtr()
		{
			return &this->data;
		}
		FeatureTypeBool& operator = (const FeatureTypeBool& other)
		{
			if (this != &other) //selfcheck
			{
				this->featureName = other.featureName;
				this->featureType = other.featureType;
				this->data = other.data;
				this->size = 1;
				this->idx = other.idx;
			}
			return *this;
		}
	};

	//Feature type "string"
	class FeatureTypeString : public FeatureType
	{
	public:
		std::string data;
		FeatureTypeString(std::string name, std::string t) : FeatureType(name, t) {}
		std::string GetDataAsString()
		{
			return this->data;
		}
		void SetData(void* data, int size = 1)
		{
			this->data = *(std::string*)data;
			this->size = size;
		}
		void CopyData(void* data, int size)
		{
			this->data = *(std::string*)data;
			this->size = size;
		}
		void* GetDataPtr()
		{
			return &this->data;
		}
		FeatureTypeString& operator = (const FeatureTypeString& other)
		{
			if (this != &other) //selfcheck
			{
				this->featureName = other.featureName;
				this->featureType = other.featureType;
				this->data = other.data;
				this->size = 1;
				this->idx = other.idx;
			}
			return *this;
		}
	};

	//Definition of iterator type
	typedef std::map<int, std::shared_ptr<FeatureType>>::iterator features_map_iter_type;
	//

	//Class which holds features from specific feature set
	class FeatureSet
	{
	public:
		std::map<int, std::shared_ptr<FeatureType>> features;
		std::string name;
		bool idx;
		FeatureSet() { 
			this->name = "";
			this->idx = false;
		}
		FeatureSet(std::string featureSetName, bool idx) : name(featureSetName), idx(idx) {  }
		~FeatureSet() {}
		//Adds new parameter to a set
		void AddFeature(int id, std::string name, std::string typeName, bool idx = false)
		{
			std::shared_ptr<FeatureType> newFeature;
			//define type
			if (typeName == "int")
				newFeature = std::make_shared<FeatureTypeInt>(name, typeName);
			else if (typeName == "float")
				newFeature = std::make_shared<FeatureTypeFloat>(name, typeName);
			else if (typeName == "double")
				newFeature = std::make_shared<FeatureTypeDouble>(name, typeName);
			else if (typeName == "bool")
				newFeature = std::make_shared<FeatureTypeBool>(name, typeName);
			else if (typeName == "string")
				newFeature = std::make_shared<FeatureTypeString>(name, typeName);

			//set idx (if the id is index or dict id)
			newFeature->idx = idx;
			//Adding new parameter set to a list
			this->features.insert(std::pair<int, std::shared_ptr<FeatureType>>(id, newFeature));
		}
		//Sets data pointer
		template<typename T>
		void SetFeatureData(int id, T *data, int size = 1)
		{
			this->features.at(id)->SetData(data, size);
		}
		//Makes a copy of the data from the pointer
		template<typename T>
		void CopyFeatureData(int id, T *data, int size = 1)
		{
			this->features.at(id)->CopyData(data, size);
		}
		FeatureSet& operator = (const FeatureSet& other)
		{
			if (this != &other) //selfcheck
			{
				this->name = other.name;
				this->idx = other.idx;
				this->features = other.features;
			}
			return *this;
		}
	};

	//Definition of iterator type
	typedef std::map<int, std::shared_ptr<FeatureSet>>::iterator featureSets_map_iter_type;
	//

	//Class which holds features from specific feature set
	class FeatureGroup
	{
	public:
		std::string name;
		bool idx;
		std::map<int, std::shared_ptr<FeatureSet>> featureSets;
		FeatureGroup() { this->name = ""; }
		FeatureGroup(std::string featureGroupName, bool idx = false) : name(featureGroupName), idx(idx) { }
		~FeatureGroup() {}
		//Adds new feature set
		void AddFeatureSet(int id, std::string name, bool idx = false)
		{
			this->featureSets.insert(std::pair<int, std::shared_ptr<FeatureSet>>(id, std::make_shared<FeatureSet>(name, idx)));
		}
		FeatureGroup& operator = (const FeatureGroup& other)
		{
			if (this != &other) //selfcheck
			{
				this->name = other.name;
				this->featureSets = other.featureSets;
				this->idx = other.idx;
			}
			return *this;
		}
	};	

	//Definition of iterator type
	typedef std::map<int, std::shared_ptr<FeatureGroup>>::iterator featureGroups_map_iter_type;
	//

	//Class which respresents a single element in the SegFile (Surfel, object, etc.)
	class SegFileElement
	{
	public:
		std::string name;
		int id;
		FeatureSet features;
		std::map<int, std::shared_ptr<FeatureSet>> featureSets;
		std::map<int, std::shared_ptr<FeatureGroup>> featureGroups;
		SegFileElement(int id, std::string name) : id(id), name(name) { this->features.name = "Base"; }
		~SegFileElement(){}
		//Adds new feature set
		void AddFeatureSet(int id, std::string name, bool idx = false)
		{
			this->featureSets.insert(std::pair<int, std::shared_ptr<FeatureSet>>(id, std::make_shared<FeatureSet>(name, idx)));
		}
		//Adds new feature group
		void AddFeatureGroup(int id, std::string name, bool idx = false)
		{
			this->featureGroups.insert(std::pair<int, std::shared_ptr<FeatureGroup>>(id, std::make_shared<FeatureGroup>(name, idx)));
		}
		SegFileElement& operator = (const SegFileElement& other)
		{
			if (this != &other) //selfcheck
			{
				this->id = other.id;
				this->name = other.name;
				this->features = other.features;
				this->featureSets = other.featureSets;
				this->featureGroups = other.featureGroups;
			}
			return *this;
		}
	};

	//Class which holds all scene elements
	class SceneSegFile
	{
	public:
		std::string name;
		std::string filename;
		std::vector<std::shared_ptr<SegFileElement>> elements;	//Should probably be a std::map but what kind of ID types to use?
		SceneSegFile(std::string name) : name(name) {}
		~SceneSegFile(){}
		//Adds new element
		void AddElement(int id, std::string name)
		{
			std::shared_ptr<SegFileElement> newElement = std::make_shared<SegFileElement>(id, name);
			//Adding new element to a list
			this->elements.push_back(newElement);
		}
		//Add element
		void AddElement(std::shared_ptr<SegFileElement> element)
		{
			//Adding element to a list
			this->elements.push_back(element);
		}

		//Saves SceneSegFile to specified filemane
		void Save(std::string filename)
		{
			//base vars
			char* tempChar;
			std::string tempStr;
			std::stringstream ss(std::stringstream::in | std::stringstream::out);
			//document
			rapidxml::xml_document<> doc;
			doc.name("Scene");
			//root node
			rapidxml::xml_node<> *rootnode = doc.allocate_node(rapidxml::node_element, "Scene");

			//saving name
			tempChar = doc.allocate_string(this->name.c_str(), this->name.size() + 1); //+1 because of the NULL terminating string
			//as atribute of root
			rootnode->append_attribute(doc.allocate_attribute("Name", tempChar));
			//adding root to document
			doc.append_node(rootnode);

			//temp nodes
			rapidxml::xml_node<> *elementNode;
			rapidxml::xml_node<> *featureSetNode;
			rapidxml::xml_node<> *featureGroupNode;
			rapidxml::xml_node<> *featureNode;

			std::shared_ptr<FeatureGroup> currFeatureGroup;
			std::shared_ptr<FeatureSet> currFeatureSet;
			std::shared_ptr<SegFileElement> currElement;
			std::shared_ptr<FeatureType> currFeature;
			//Iterating through elements
			for (int i = 0; i < this->elements.size(); i++)
			{
				currElement = this->elements.at(i);

				//Creating new element set node
				elementNode = doc.allocate_node(rapidxml::node_element, "Element");
				//Adding atributes name and id
				tempChar = doc.allocate_string(currElement->name.c_str(), currElement->name.size() + 1); //+1 because of the NULL terminating string
				elementNode->append_attribute(doc.allocate_attribute("Name", tempChar));
				ss.clear();
				ss.str("");
				ss << currElement->id;
				tempChar = doc.allocate_string(ss.str().c_str(), ss.str().size() + 1); //+1 because of the NULL terminating string
				elementNode->append_attribute(doc.allocate_attribute("idx", tempChar));	//elements have indices
				//
				//Iterating through feature list
				for (features_map_iter_type featureIterator = currElement->features.features.begin(); featureIterator != currElement->features.features.end(); featureIterator++)
				{
					// iterator->first = key
					// iterator->second = value
					currFeature = featureIterator->second;
					AddFeatureToXMLNode(elementNode, &doc, currFeature, featureIterator->first);
				}
				//Iterating through feature sets and their lists of features
				for (featureSets_map_iter_type featureSetsIterator = currElement->featureSets.begin(); featureSetsIterator != currElement->featureSets.end(); featureSetsIterator++)
				{
					// iterator->first = key
					// iterator->second = value
					currFeatureSet = featureSetsIterator->second;
					//Creating new feature set node
					featureSetNode = doc.allocate_node(rapidxml::node_element, "FeatureSet");
					//Adding atributes: name and id
					tempChar = doc.allocate_string(currFeatureSet->name.c_str(), currFeatureSet->name.size() + 1); //+1 because of the NULL terminating string
					featureSetNode->append_attribute(doc.allocate_attribute("Name", tempChar));
					//
					ss.clear();
					ss.str("");
					ss << featureSetsIterator->first;
					tempChar = doc.allocate_string(ss.str().c_str(), ss.str().size() + 1); //+1 because of the NULL terminating string
					if (currFeatureSet->idx)
						featureSetNode->append_attribute(doc.allocate_attribute("idx", tempChar));
					else
						featureSetNode->append_attribute(doc.allocate_attribute("idd", tempChar));
					//Iterating through feature list
					for (features_map_iter_type featureIterator = currFeatureSet->features.begin(); featureIterator != currFeatureSet->features.end(); featureIterator++)
					{
						// iterator->first = key
						// iterator->second = value
						currFeature = featureIterator->second;
						AddFeatureToXMLNode(featureSetNode, &doc, currFeature, featureIterator->first);
					}
					//Appending featureset to element
					elementNode->append_node(featureSetNode);
				}
				//Iterating through feature groups and their list of feature sets and features
				for (featureGroups_map_iter_type featureGroupsIterator = currElement->featureGroups.begin(); featureGroupsIterator != currElement->featureGroups.end(); featureGroupsIterator++)
				{
					// iterator->first = key
					// iterator->second = value
					currFeatureGroup = featureGroupsIterator->second;
					//Creating new feature group node
					featureGroupNode = doc.allocate_node(rapidxml::node_element, "FeatureGroup");
					//Adding atributes: name and id
					tempChar = doc.allocate_string(currFeatureGroup->name.c_str(), currFeatureGroup->name.size() + 1); //+1 because of the NULL terminating string
					featureGroupNode->append_attribute(doc.allocate_attribute("Name", tempChar));
					//
					ss.clear();
					ss.str("");
					ss << featureGroupsIterator->first;
					tempChar = doc.allocate_string(ss.str().c_str(), ss.str().size() + 1); //+1 because of the NULL terminating string
					if (currFeatureGroup->idx)
						featureGroupNode->append_attribute(doc.allocate_attribute("idx", tempChar));
					else
						featureGroupNode->append_attribute(doc.allocate_attribute("idd", tempChar));
					//Iterating through feature sets and their lists of features
					for (featureSets_map_iter_type featureSetsIterator = currFeatureGroup->featureSets.begin(); featureSetsIterator != currFeatureGroup->featureSets.end(); featureSetsIterator++)
					{
						// iterator->first = key
						// iterator->second = value
						currFeatureSet = featureSetsIterator->second;
						//Creating new feature set node
						featureSetNode = doc.allocate_node(rapidxml::node_element, "FeatureSet");
						//Adding atributes: name and id
						tempChar = doc.allocate_string(currFeatureSet->name.c_str(), currFeatureSet->name.size() + 1); //+1 because of the NULL terminating string
						featureSetNode->append_attribute(doc.allocate_attribute("Name", tempChar));
						//
						ss.clear();
						ss.str("");
						ss << featureSetsIterator->first;
						tempChar = doc.allocate_string(ss.str().c_str(), ss.str().size() + 1); //+1 because of the NULL terminating string
						if (currFeatureSet->idx)
							featureSetNode->append_attribute(doc.allocate_attribute("idx", tempChar));
						else
							featureSetNode->append_attribute(doc.allocate_attribute("idd", tempChar));
						//Iterating through feature list
						for (features_map_iter_type featureIterator = currFeatureSet->features.begin(); featureIterator != currFeatureSet->features.end(); featureIterator++)
						{
							// iterator->first = key
							// iterator->second = value
							currFeature = featureIterator->second;
							AddFeatureToXMLNode(featureSetNode, &doc, currFeature, featureIterator->first);
						}
						//Appending featureset to group
						featureGroupNode->append_node(featureSetNode);
					}
					//Appending feature group to element
					elementNode->append_node(featureGroupNode);
				}
				//Appending element to root
				rootnode->append_node(elementNode);
			}
			//Saving to file
			std::ofstream dat(filename.c_str());
			dat << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
			dat << doc;
			dat.close();

			this->filename = filename;
		}

		//Load SceneSegFile from specified filename
		void Load(std::string filename)
		{
			//temp vars
			std::stringstream ss(std::stringstream::in | std::stringstream::out);
			std::string tempString;

			//xml objects
			rapidxml::xml_document<> doc;
			rapidxml::xml_node<> *rootnode;
			rapidxml::xml_node<> *elementNode;
			rapidxml::xml_node<> *elementChildNode;
			rapidxml::xml_node<> *featureSetChildNode;
			rapidxml::xml_node<> *featureGroupChildNode;

			//Load file
			std::ifstream dat(filename.c_str());
			if (!dat)
				return;
			this->filename = filename;

			//Load whole file to char array
			std::string str((std::istreambuf_iterator<char>(dat)), std::istreambuf_iterator<char>());
			char *text = new char[str.size() + 1];
			strcpy(text, str.c_str());
			dat.close();

			//Parse the XML file
			doc.parse<0>(text);

			//root node of the document
			rootnode = doc.first_node();
			//name
			ss.clear();
			ss.str("");
			tempString = std::string(rootnode->first_attribute()->value(), rootnode->first_attribute()->value_size());
			this->name = tempString;

			//Iterating through parameter sets
			//First root child node (first parameter set)
			elementNode = rootnode->first_node();
			while (elementNode)
			{
				//Getting element atributes
				std::string elementName;
				int elementID;
				//There are only two atributes so we can use FIRST and LAST
				elementName = elementNode->first_attribute()->value();
				//
				ss.clear();
				ss.str("");
				ss << elementNode->last_attribute()->value();
				ss >> elementID;

				//Creating new parameter set
				std::shared_ptr<SegFileElement> newElement = std::make_shared<SegFileElement>(elementID, elementName);

				//Iterating through child nodes of current element
				//First parameter set child node (first parameter)
				elementChildNode = elementNode->first_node();
				while (elementChildNode)
				{
					//Getting child atributes
					std::string childName;
					std::string childName2;
					std::string featureType;
					std::string nodeName;
					std::string atribName;
					int childID;
					int childID2;
					bool childID2Idx = false;
					bool isFeatureGroup = false;
					bool isFeatureGroupIdx = false;
					bool isFeatureSet = false;
					bool isFeatureSetIdx = false;
					bool isFeature = false;
					bool isFeatureIdx = false;
					nodeName = elementChildNode->name();
					if (nodeName == "Feature")	//If it is feature 
					{
						//There are three atributes so we can use FIRST, FIRST->NEXT and LAST
						childName = elementChildNode->first_attribute()->value();
						//
						featureType = elementChildNode->first_attribute()->next_attribute()->value();
						//
						ss.clear();
						ss.str("");
						ss << elementChildNode->last_attribute()->value();
						ss >> childID;
						//						
						isFeature = true;
						atribName = elementChildNode->last_attribute()->name();
						if (atribName == "idx")
							isFeatureIdx = true;
					}
					else if (nodeName == "FeatureSet")
					{
						//There are only two atributes
						childName = elementChildNode->first_attribute()->value();
						ss.clear();
						ss.str("");
						ss << elementChildNode->last_attribute()->value();
						ss >> childID;
						//					
						isFeatureSet = true;
						atribName = elementChildNode->last_attribute()->name();
						if (atribName == "idx")
							isFeatureSetIdx = true;
					}
					else if (nodeName == "FeatureGroup")
					{
						//There are only two atributes
						childName = elementChildNode->first_attribute()->value();
						ss.clear();
						ss.str("");
						ss << elementChildNode->last_attribute()->value();
						ss >> childID;
						//					
						isFeatureGroup = true;
						atribName = elementChildNode->last_attribute()->name();
						if (atribName == "idx")
							isFeatureGroupIdx = true;
					}

					//Check
					if (isFeatureGroup)
					{
						std::shared_ptr<FeatureGroup> newFeatureGroup = std::make_shared<FeatureGroup>(childName, isFeatureGroupIdx);
						featureGroupChildNode = elementChildNode->first_node();
						while (featureGroupChildNode)
						{
							childName2 = featureGroupChildNode->first_attribute()->value();
							ss.clear();
							ss.str("");
							ss << featureGroupChildNode->last_attribute()->value();
							ss >> childID2;
							atribName = featureGroupChildNode->last_attribute()->name();
							if (atribName == "idx")
								childID2Idx = true;
							else
								childID2Idx = false;

							std::shared_ptr<FeatureSet> newFeatureSet = std::make_shared<FeatureSet>(childName2, childID2Idx);
							//iterating trough all features in the set
							featureSetChildNode = featureGroupChildNode->first_node();
							while (featureSetChildNode)
							{
								AddFeatureToSet(featureSetChildNode, newFeatureSet.get());
								//next feature
								featureSetChildNode = featureSetChildNode->next_sibling();
							}
							//Adding new feature set to group
							newFeatureGroup->featureSets.insert(std::pair<int, std::shared_ptr<FeatureSet>>(childID2, newFeatureSet));
							//next feature set
							featureGroupChildNode = featureGroupChildNode->next_sibling();
						}
						
						//Adding new feature group to element
						newElement->featureGroups.insert(std::pair<int, std::shared_ptr<FeatureGroup>>(childID, newFeatureGroup));
					}
					else if (isFeatureSet)
					{
						std::shared_ptr<FeatureSet> newFeatureSet = std::make_shared<FeatureSet>(childName, isFeatureSetIdx);
						//iterating trough all features in the set
						featureSetChildNode = elementChildNode->first_node();
						while (featureSetChildNode)
						{
							AddFeatureToSet(featureSetChildNode, newFeatureSet.get());
							//next feature
							featureSetChildNode = featureSetChildNode->next_sibling();
						}
						//Adding new feature set to element
						newElement->featureSets.insert(std::pair<int, std::shared_ptr<FeatureSet>>(childID, newFeatureSet));
					}
					else if (isFeature)
						AddFeatureToSet(elementChildNode, &newElement->features);

					//next child
					elementChildNode = elementChildNode->next_sibling();
				}

				//Adding new parameter set to list
				this->elements.push_back(newElement);

				//next parameter set
				elementNode = elementNode->next_sibling();
			}

			//Releasing memory
			delete[] text;
		}
	private:
		void AddFeatureToSet(rapidxml::xml_node<> *node, FeatureSet *featureSet)
		{
			//temp vars
			std::stringstream ss(std::stringstream::in | std::stringstream::out);
			std::string tempString;

			//iterating trough all features in the set
			std::string setFeatureName;
			std::string setFeatureType;
			std::string atribName;
			int setFeatureID;
			bool setFeatureIdx = false;
			//There are only three atributes so we can use FIRST, FIRST->NEXT and LAST
			setFeatureName = node->first_attribute()->value();
			//
			setFeatureType = node->first_attribute()->next_attribute()->value();
			//
			ss.clear();
			ss.str("");
			ss << node->last_attribute()->value();
			ss >> setFeatureID;
			atribName = node->last_attribute()->name();
			if (atribName == "idx")
				setFeatureIdx = true;
			//Creating appropriate feature type
			if (setFeatureType == "int")
			{
				std::shared_ptr<FeatureTypeInt> featureTypeInt = std::make_shared<FeatureTypeInt>(setFeatureName, setFeatureType);
				int featureVal;
				ss.clear();
				ss.str("");
				ss << node->value();
				//Reading values until end of stream
				std::vector<int> intVector;
				do
				{
					ss >> featureVal;
					intVector.push_back(featureVal);
				} while (!ss.eof());
				//create data array
				int *intData = new int[intVector.size()];
				for (int i = 0; i < intVector.size(); i++)
					intData[i] = intVector[i];
				featureTypeInt->SetData(intData, intVector.size());
				featureTypeInt->idx = setFeatureIdx;
				//Adding new feature to a set
				featureSet->features.insert(std::pair<int, std::shared_ptr<FeatureType>>(setFeatureID, featureTypeInt));
			}
			else if (setFeatureType == "float")
			{
				std::shared_ptr<FeatureTypeFloat> featureTypeFloat = std::make_shared<FeatureTypeFloat>(setFeatureName, setFeatureType);
				float featureVal;
				ss.clear();
				ss.str("");
				ss << node->value();
				//Reading values until end of stream
				std::vector<float> floatVector;
				do
				{
					ss >> featureVal;
					floatVector.push_back(featureVal);
				} while (!ss.eof());
				//create data array
				float *floatData = new float[floatVector.size()];
				for (int i = 0; i < floatVector.size(); i++)
					floatData[i] = floatVector[i];
				featureTypeFloat->SetData(floatData, floatVector.size());
				featureTypeFloat->idx = setFeatureIdx;
				//Adding new feature to a set
				featureSet->features.insert(std::pair<int, std::shared_ptr<FeatureType>>(setFeatureID, featureTypeFloat));
			}
			else if (setFeatureType == "double")
			{
				std::shared_ptr<FeatureTypeDouble> featureTypeDouble = std::make_shared<FeatureTypeDouble>(setFeatureName, setFeatureType);
				double featureVal;
				ss.clear();
				ss.str("");
				ss << node->value();
				//Reading values until end of stream
				std::vector<double> doubleVector;
				do
				{
					ss >> featureVal;
					doubleVector.push_back(featureVal);
				} while (!ss.eof());
				//create data array
				double *doubleData = new double[doubleVector.size()];
				for (int i = 0; i < doubleVector.size(); i++)
					doubleData[i] = doubleVector[i];
				featureTypeDouble->SetData(doubleData, doubleVector.size());
				featureTypeDouble->idx = setFeatureIdx;
				//Adding new feature to a set
				featureSet->features.insert(std::pair<int, std::shared_ptr<FeatureType>>(setFeatureID, featureTypeDouble));
			}
			else if (setFeatureType == "string")
			{
				std::shared_ptr<FeatureTypeString> featureTypeString = std::make_shared<FeatureTypeString>(setFeatureName, setFeatureType);
				ss.clear();
				ss.str("");
				ss << node->value();
				featureTypeString->data = ss.str();
				featureTypeString->idx = setFeatureIdx;
				//Adding new parameter to a set
				featureSet->features.insert(std::pair<int, std::shared_ptr<FeatureType>>(setFeatureID, featureTypeString));
			}
			else if (setFeatureType == "bool")
			{
				std::shared_ptr<FeatureTypeBool> featureTypeBool = std::make_shared<FeatureTypeBool>(setFeatureName, setFeatureType);
				ss.clear();
				ss.str("");
				ss << node->value();
				ss >> featureTypeBool->data;
				featureTypeBool->idx = setFeatureIdx;
				//Adding new parameter to a set
				featureSet->features.insert(std::pair<int, std::shared_ptr<FeatureType>>(setFeatureID, featureTypeBool));
			}
		}

		void AddFeatureToXMLNode(rapidxml::xml_node<> *node, rapidxml::xml_document<> *doc, std::shared_ptr<FeatureType> feature, int id)
		{
			//base vars
			char* tempChar;
			std::string tempStr;
			std::stringstream ss(std::stringstream::in | std::stringstream::out);
			rapidxml::xml_node<> *featureNode;

			//Creating new feature node
			featureNode = doc->allocate_node(rapidxml::node_element, "Feature");
			//Adding atributes: name, type and id
			tempChar = doc->allocate_string(feature->featureName.c_str(), feature->featureName.size() + 1); //+1 because of the NULL terminating string
			featureNode->append_attribute(doc->allocate_attribute("Name", tempChar));
			//
			tempChar = doc->allocate_string(feature->featureType.c_str(), feature->featureType.size() + 1); //+1 because of the NULL terminating string
			featureNode->append_attribute(doc->allocate_attribute("Type", tempChar));
			//
			ss.clear();
			ss.str("");
			ss << id;
			tempChar = doc->allocate_string(ss.str().c_str(), ss.str().size() + 1); //+1 because of the NULL terminating string
			if (feature->idx)
				featureNode->append_attribute(doc->allocate_attribute("idx", tempChar));
			else
				featureNode->append_attribute(doc->allocate_attribute("idd", tempChar));
			//Adding value
			std::string featureValue = feature->GetDataAsString();
			tempChar = doc->allocate_string(featureValue.c_str(), featureValue.size() + 1);
			featureNode->value(tempChar);
			//Appending feature
			node->append_node(featureNode);
		}
	};

	//Registered feature groups
	//Dictionary class for list of feature groups
	class FeatureGroupsDictionary
	{
	public:
		static std::map<int, std::string> dictionary;
	};
	class FeatureGroupsList
	{
	public:
		enum list
		{
			AdjacencyFeatureGroup = 0,
		};
	};

	//Registered feature sets
	//Dictionary class for list of feature sets
	class FeatureSetsDictionary
	{
	public:
		static std::map<int, std::string> dictionary;
	};
	class FeatureSetsList
	{
	public:
		enum list
		{
			AdjacencyNode = 0,
		};
	};

	//Registered features
	//Dictionary class for application's general parameters
	class FeaturesDictionary
	{
	public:
		static std::map<int, std::string> dictionary;
	};
	class FeaturesList
	{
	public:
		enum list
		{
			Centroid = 0,	//float
			NormalAngleDifference,	//float
			CupysFeature,	//double
			GTObjectID,		//int
			SameGTObject,	//bool
			PixelAffiliation,	//int
			GTObjHistogram,	//int
			CommonBoundaryLength,	//int
			Vertices3D, //float
			FalseSegmentationCost,	//int
		};
	};

	SceneSegFile* GenerateTestSceneSegFile();
}
