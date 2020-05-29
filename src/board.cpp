#include "board.h"
#include <array>


bool asc(const Mass* o1, const Mass* o2) {
	return o1->getCast() < o2->getCast();
}
bool Board::find(const Point& start, const Point& goal)
{
	Mass& mass_start = getMass(start);
	Mass& mass_goal = getMass(goal);

	mass_start.setStatus(Mass::START);
	mass_goal.setStatus(Mass::GOAL);

	open_list_.clear();
	open_list_.push_back(&mass_start);

	while (!open_list_.empty())// オープンリストが空でない
	{
		// 現在のノード=オープンリストの最も安価なリスト
		std::sort(open_list_.begin(), open_list_.end(), asc);
		auto it = open_list_.begin();
		Mass* current = *it;

		if (current->getStatus() == Mass::GOAL)
		{
			// 目的地なら経路完成
			Mass* p = current;
			while (p)
			{
				if (p->getStatus() == Mass::Mass::BLANK)
					p->setStatus(Mass::WAYPOINT);
				p = p->getParent();
			}
			return true;
		}
		else
		{
			// 現在のノードをクローズドリストに戻す
			open_list_.erase(it);
			current->setListed(Mass::CLOSE);
			// 現在のノードの隣接する各ノードを調べる
			const Point& pos = current->getPos();
			Point next[4] = { pos.getRight(),pos.getLeft(),pos.getUp(),pos.getDown() };
			for (auto& c : next)
			{
				if (c.x() < 0 || BOARD_SIZE <= c.x())continue;
				if (c.y() < 0 || BOARD_SIZE <= c.y())continue;
				Mass& m = getMass(c);
				if (!m.isListed(Mass::OPEN) &&
					!m.isListed(Mass::CLOSE) &&
					m.getStatus() != Mass::WALL)// オープンリスト、クローズドリストに含まれていない、障害物ではない
				{
					// オープンリストに移してコスト計算する
					open_list_.push_back(&m);
					m.setParent(current, goal);
					m.setListed(Mass::OPEN);
				}
			}
		}

	}
	return false;
}

void Board::show() const 
{
	std::cout << std::endl;

	for (int y = 0; y < BOARD_SIZE; y++) {
		std::cout << " ";
		for (int x = 0; x < BOARD_SIZE; x++) {
			std::cout << "+-";
		}
		std::cout << "+" << std::endl;

		std::cout << " ";
		for (int x = 0; x < BOARD_SIZE; x++) {
			std::cout << "|";
			switch (mass_[y][x].getStatus()) {
			case Mass::BLANK:
				std::cout << " ";
				break;
			case Mass::START:
				std::cout << "S";
				break;
			case Mass::GOAL:
				std::cout << "G";
				break;
			case Mass::WAYPOINT:
				std::cout << "o";
				break;
			case Mass::WALL:
				std::cout << "#";
				break;
			case Mass::WATER:
				std::cout << "~";
				break;
			case Mass::ROAD:
				std::cout << "$";
				break;
			}
		}
		std::cout << "|" << std::endl;
	}

	std::cout << " ";
	for (int x = 0; x < BOARD_SIZE; x++) {
		std::cout << "+-";
	}
	std::cout << "+" << std::endl;

}


