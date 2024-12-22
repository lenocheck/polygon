#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
    int splitArray(vector<int>& nums, int k) { // временная сложность - O(nlog(sum), пространственная сложность - O(1), где n - кол-во элементов в nums, sum — это сумма всех элементов в массиве
        int left = findMax(nums); // временная сложность - O(n), пространственная сложность - O(1), где n - кол-во элементов в nums
        int right = findSum(nums); // временная сложность - O(n), пространственная сложность - O(1), где n - кол-во элементов в nums

        while (left < right) { // временная сложность - O(nlog(sum), пространственная сложность - O(1), где n - кол-во элементов в nums, sum — это сумма всех элементов в массиве
            int mid = (left + right) / 2;

            if (canSplit(nums, k, mid)) { // временная сложность - O(n), пространственная сложность - O(1), где n - кол-во элементов в nums
                right = mid;
            } else {
                left = mid + 1;
            }
        }

        return left;
    }

private:
    int findMax(vector<int> nums) { // временная сложность - O(n), пространственная сложность - O(1), где n - кол-во элементов в nums
        int maxVal = nums[0];
        for (int num : nums) {
            if (num > maxVal) {
                maxVal = num;
            }
        }
        return maxVal;
    }

    int findSum(vector<int> nums) { // временная сложность - O(n), пространственная сложность - O(1), где n - кол-во элементов в nums
        int totalSum = 0;
        for (int num : nums) {
            totalSum += num;
        }
        return totalSum;
    }

    bool canSplit(vector<int> nums, int k, int maxSum) { // временная сложность - O(n), пространственная сложность - O(1), где n - кол-во элементов в nums
        int count = 1;
        int currentSum = 0;

        for (int num : nums) { // временная сложность - O(n), пространственная сложность - O(1), где n - кол-во элементов в nums
            currentSum += num;

            if (currentSum > maxSum) {
                count++;
                currentSum = num;
            }

            if (count > k) {
                return false;
            }
        }

        return true;
    }
};


int main() {
    vector<int> nums = {1, 2, 3, 4, 5};
    int k = 2;
    Solution solution;
    int result = solution.splitArray(nums, k);
    cout << result;
    return 0;
}
