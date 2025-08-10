20. Valid Parentheses

Given a string s containing just the characters '(', ')', '{', '}', '[' and ']', determine if the input string is valid.

An input string is valid if:

Open brackets must be closed by the same type of brackets.
Open brackets must be closed in the correct order.
Every close bracket has a corresponding open bracket of the same type.
 

Example 1:

Input: s = "()"

Output: true

Example 2:

Input: s = "()[]{}"

Output: true

```cpp
class Solution {
public:
    bool isValid(string s) {
        stack<char>st;
        int n = s.length();
        for(int i =0;i<n;i++) {
            if(s[i] == '(' || s[i] == '[' || s[i] =='{') {
                st.push(s[i]);
            } else {
                if(st.empty() || (st.top() == '(' && s[i]!=')') ||
                (st.top() == '[' && s[i]!=']') || (st.top() == '{' && s[i]!='}')) {
                    return false;
                }
                st.pop();
            }
        }
        return st.empty();
        

    }
};
```

✅ Time Complexity (TC):
cpp
Copy
Edit
for(int i = 0; i < n; i++) {
    // Each character is processed once
}
You iterate through the string once.

Stack operations (push, pop, top) all take O(1) time.

✅ Time Complexity = O(n)
(where n is the length of the input string)

✅ Space Complexity (SC):
In the worst case (e.g., "((((((("), the stack stores all characters.

So space used by the stack is proportional to the number of opening brackets.

✅ Space Complexity = O(n)
(where n is the length of the input string)

✅ Final Summary:
Complexity Type	Value
Time Complexity	O(n)
Space Complexity	O(n)