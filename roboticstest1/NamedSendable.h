1	+/*
2	+ * NamedSendable.h
3	+ *
4	+ *  Created on: Oct 19, 2012
5	+ *      Author: Mitchell Wills
6	+ */
7	
8	#ifndef NAMEDSENDABLE_H_
9	#define NAMEDSENDABLE_H_
10	+
11	+
12	#include <string>
13	#include "SmartDashboard/Sendable.h"
14	+
15	+/**
16	+ * The interface for sendable objects that gives the sendable a default name in the Smart Dashboard
17	+ * 
18	+ */
19	+class NamedSendable : public Sendable
20	+{
21	+public:
22	+
23	+    /**
24	+     * @return the name of the subtable of SmartDashboard that the Sendable object will use
25	+     */
26	+	virtual std::string GetName() = 0;
27	+};
28	+
29	+#endif /* NAMEDSENDABLE_H_ */
 