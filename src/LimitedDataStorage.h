#pragma once

#include <QMap>
#include <QList>
#include <QVector>
/*
 * TODO: add removing strategy, add insert/gc strategy
 */
template<typename KeyT, typename DataT, typename DestroyStrategy>
class LimitedDataStorage
{
public:
  LimitedDataStorage(int MAX_NUM);
  virtual ~LimitedDataStorage();

  void insert(KeyT key, DataT data);
  void remove(KeyT key);
  void removeFirst();

  bool contains(KeyT key) const;
  DataT data(KeyT key) const;
  QList<KeyT> keys() const;
  int order(KeyT key) const;
  bool isEmpty() const;
  bool isFull() const;
  int size() const;

protected:
  int MAX_NUM_;
  QMap<KeyT, DataT> data_;
  QVector<KeyT> insertion_order_;
};

template<typename KeyT, typename DataT, typename DestroyStrategy>
QList<KeyT> LimitedDataStorage<KeyT, DataT, DestroyStrategy>::keys() const
{
  return data_.keys();
}

template<typename KeyT, typename DataT, typename DestroyStrategy>
bool LimitedDataStorage<KeyT, DataT, DestroyStrategy>::isFull() const
{
  return data_.size() >= MAX_NUM_;
}

template<typename KeyT, typename DataT, typename DestroyStrategy>
int LimitedDataStorage<KeyT, DataT, DestroyStrategy>::order(KeyT key) const
{
  return insertion_order_.indexOf(key);
}

template<typename KeyT, typename DataT, typename DestroyStrategy>
LimitedDataStorage<KeyT, DataT, DestroyStrategy>::LimitedDataStorage(int MAX_NUM)
  : MAX_NUM_(MAX_NUM)
{

}

template<typename KeyT, typename DataT, typename DestroyStrategy>
LimitedDataStorage<KeyT, DataT, DestroyStrategy>::~LimitedDataStorage()
{
  data_.clear();
  insertion_order_.clear();
}

template<typename KeyT, typename DataT, typename DestroyStrategy>
int LimitedDataStorage<KeyT, DataT, DestroyStrategy>::size() const
{
  return data_.size();
}

template<typename KeyT, typename DataT, typename DestroyStrategy>
bool LimitedDataStorage<KeyT, DataT, DestroyStrategy>::isEmpty() const
{
  return size() == 0;
}

template<typename KeyT, typename DataT, typename DestroyStrategy>
bool LimitedDataStorage<KeyT, DataT, DestroyStrategy>::contains(KeyT key) const
{
  return data_.contains(key);
}

template<typename KeyT, typename DataT, typename DestroyStrategy>
void LimitedDataStorage<KeyT, DataT, DestroyStrategy>::remove(KeyT key)
{
  DestroyStrategy::onDestroy(data_[key]);
  data_.remove(key);
  insertion_order_.removeAll(key);
}

template<typename KeyT, typename DataT, typename DestroyStrategy>
void LimitedDataStorage<KeyT, DataT, DestroyStrategy>::removeFirst()
{
  if (insertion_order_.isEmpty()) return;
  KeyT key = insertion_order_.takeFirst();
  DestroyStrategy::onDestroy(data_[key]);
  data_.remove(key);
}

template<typename KeyT, typename DataT, typename DestroyStrategy>
void LimitedDataStorage<KeyT, DataT, DestroyStrategy>::insert(KeyT key, DataT data)
{
  if (data_.size() == MAX_NUM_) removeFirst();
  data_.insert(key, data);
  insertion_order_.push_back(key);
}

template<typename KeyT, typename DataT, typename DestroyStrategy>
DataT LimitedDataStorage<KeyT, DataT, DestroyStrategy>::data(KeyT key) const
{
  return data_[key];
}
