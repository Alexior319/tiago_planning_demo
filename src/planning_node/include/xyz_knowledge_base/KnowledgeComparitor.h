#include <ros/ros.h>
#include <vector>
#include "xyz_knowledge_msgs/KnowledgeItem.h"
#include "xyz_knowledge_msgs/DomainInequality.h"
#include "xyz_knowledge_msgs/ExprBase.h"
#include "xyz_knowledge_msgs/ExprComposite.h"

#ifndef KCL_knowledgecomparitor
#define KCL_knowledgecomparitor

namespace KCL_rosplan {

	class KnowledgeComparitor
	{
	public:
		static double getValue(const xyz_knowledge_msgs::ExprBase &expr, const std::vector<xyz_knowledge_msgs::KnowledgeItem> &modelFunctions);
		static double evaluateOperation(const double &lhs, const double &rhs, const xyz_knowledge_msgs::ExprBase &op);
		static double evaluateExpression(const xyz_knowledge_msgs::ExprComposite &a, const std::vector<xyz_knowledge_msgs::KnowledgeItem> &modelFunctions);
		static bool inequalityTrue(const xyz_knowledge_msgs::KnowledgeItem &a, const std::vector<xyz_knowledge_msgs::KnowledgeItem> &modelFunctions);
		static bool containsKnowledge(const xyz_knowledge_msgs::KnowledgeItem &a, const xyz_knowledge_msgs::KnowledgeItem &b);
		static bool containsInstance(const xyz_knowledge_msgs::KnowledgeItem &a, std::string &name);
	};
}
#endif
