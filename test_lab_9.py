import unittest
import numpy as np
from lab_9 import replace_with_counts


class TestReplaceWithCounts(unittest.TestCase):

    def test_replace_with_counts(self):
        # тест 1: обычный случай
        arr_1 = np.array([1, 2, 2, 3, 3, 3, 4, 4, 4, 4])
        result_1 = replace_with_counts(arr_1)
        expected_1 = np.array([1, 2, 2, 3, 3, 3, 4, 4, 4, 4])
        np.testing.assert_array_equal(result_1, expected_1)

        # тест 2: все элементы одинаковы
        arr_2 = np.array([5, 5, 5, 5])
        result_2 = replace_with_counts(arr_2)
        expected_2 = np.array([4, 4, 4, 4])  # все элементы встречаются 4 раза
        np.testing.assert_array_equal(result_2, expected_2)

        # тест 3: все элементы уникальны
        arr_3 = np.array([1, 2, 3, 4])
        result_3 = replace_with_counts(arr_3)
        expected_3 = np.array([1, 1, 1, 1])  # каждый элемент встречается 1 раз
        np.testing.assert_array_equal(result_3, expected_3)

        # тест 4: массив с повторяющимися нулями
        arr_4 = np.array([0, 1, 2, 0, 1])
        result_4 = replace_with_counts(arr_4)
        expected_4 = np.array([2, 2, 1, 2, 2])  # 0 встречается 2 раза, 1 встречается 2 раза, 2 — 1 раз
        np.testing.assert_array_equal(result_4, expected_4)

        # тест 5: пустой массив
        arr_5 = np.array([])
        result_5 = replace_with_counts(arr_5)
        expected_5 = np.array([])  # пустой массив должен вернуть пустой массив
        np.testing.assert_array_equal(result_5, expected_5)

        # тест 6: все элементы нулевые
        arr_6 = np.zeros(4, dtype=int)
        result_6 = replace_with_counts(arr_6)
        expected_6 = np.array([4, 4, 4, 4])  # все элементы встречаются 4 раза
        np.testing.assert_array_equal(result_6, expected_6)

        # тест 7: в массиве есть отрицательный элемент (ожидается ошибка)
        arr_7 = np.array([1, 2, -3, 4])
        with self.assertRaises(ValueError):
            replace_with_counts(arr_7)


if __name__ == "__main__":
    unittest.main()
