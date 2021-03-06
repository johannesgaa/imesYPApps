package de.unihannover.imes.imesyoubotshow;

import de.unihannover.imes.imesyoubotshow.dummy.DummyContent;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.support.v4.app.FragmentActivity;
import android.view.View;


/**
 * An activity representing a list of Demos. This activity has different
 * presentations for handset and tablet-size devices. On handsets, the activity
 * presents a list of items, which when touched, lead to a
 * {@link DemoDetailActivity} representing item details. On tablets, the
 * activity presents the list of items and item details side-by-side using two
 * vertical panes.
 * <p>
 * The activity makes heavy use of fragments. The list of items is a
 * {@link DemoListFragment} and the item details (if present) is a
 * {@link DemoDetailFragment}.
 * <p>
 * This activity also implements the required {@link DemoListFragment.Callbacks}
 * interface to listen for item selections.
 */
public class DemoListActivity extends FragmentActivity implements
		DemoListFragment.Callbacks {

	/**
	 * Whether or not the activity is in two-pane mode, i.e. running on a tablet
	 * device.
	 */
	private boolean mTwoPane;
	private int mSelectedID;
	

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_demo_list);

		if (findViewById(R.id.demo_detail_container) != null) {
			// The detail container view will be present only in the
			// large-screen layouts (res/values-large and
			// res/values-sw600dp). If this view is present, then the
			// activity should be in two-pane mode.
			mTwoPane = true;

			// In two-pane mode, list items should be given the
			// 'activated' state when touched.
			((DemoListFragment) getSupportFragmentManager().findFragmentById(
					R.id.demo_list)).setActivateOnItemClick(true);
		}

		// TODO: If exposing deep links into your app, handle intents here.
	}

	/**
	 * Callback method from {@link DemoListFragment.Callbacks} indicating that
	 * the item with the given ID was selected.
	 */
	@Override
	public void onItemSelected(String id) {
		if (mTwoPane) {
			// In two-pane mode, show the detail view in this activity by
			// adding or replacing the detail fragment using a
			// fragment transaction.
			Bundle arguments = new Bundle();
			arguments.putString(DemoDetailFragment.ARG_ITEM_ID, id);
			DemoDetailFragment fragment = new DemoDetailFragment();
			fragment.setArguments(arguments);
			getSupportFragmentManager().beginTransaction()
					.replace(R.id.demo_detail_container, fragment).commit();
			mSelectedID = Integer.parseInt(id);;

		} else {
			// In single-pane mode, simply start the detail activity
			// for the selected item ID.
			Intent detailIntent = new Intent(this, DemoDetailActivity.class);
			detailIntent.putExtra(DemoDetailFragment.ARG_ITEM_ID, id);
			startActivity(detailIntent);
		}
	}
	
	public void OnIMESLogo(View view){
	    Uri uriUrl = Uri.parse("http://www.imes.uni-hannover.de/");  
	    Intent launchBrowser = new Intent(Intent.ACTION_VIEW, uriUrl);  
	    startActivity(launchBrowser);  
	
	}
	
	public void OnLUHLogo(View view){
		 Uri uriUrl = Uri.parse("http://www.uni-hannover.de/");  
         Intent launchBrowser = new Intent(Intent.ACTION_VIEW, uriUrl);  
		 startActivity(launchBrowser);  
		
	}
	
	public void OnRun(View view){
		
		//switchcase fuer alle activities
		switch(mSelectedID)
		{
		case 1:
			Intent armmotiondemo = new Intent(this, ImesArmMotionDemo.class);
			startActivity(armmotiondemo);
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
		case 5:
			break;
		case 6:
			break;
		case 7:
			break;
		case 8:
			break;
		default:
			break;
		
		}
		
		
		
		
	}
}
